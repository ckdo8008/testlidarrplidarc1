import 'dart:async';
import 'dart:isolate';
import 'dart:math' as math;
import 'dart:typed_data';
import 'package:flutter/foundation.dart';
import 'package:flutter/cupertino.dart';
import 'package:usb_serial/usb_serial.dart';

/// 공개: 원시 프레임(객체 생성 없이)
class RplidarFrame {
  final Float32List angles; // degrees
  final Float32List dists;  // mm
  final Uint8List quals;    // 0..255
  const RplidarFrame(this.angles, this.dists, this.quals);
  int get length => angles.length;
}

/// (레거시 호환이 필요하면 유지)
class RplidarPoint {
  final double angleDeg;   // 0..360
  final double distanceMm; // mm
  final int quality;       // 0..255
  final bool startFlag;    // 의미 없음(프레임 결과)
  RplidarPoint(this.angleDeg, this.distanceMm, this.quality, this.startFlag);
}

class RplidarC1 {
  static const int baud = 460800;
  static const List<int> _CMD_SCAN = [0xA5, 0x20];
  static const List<int> _CMD_STOP = [0xA5, 0x25];
  static const List<int> _CMD_GET_INFO = [0xA5, 0x50];
  static const List<int> _CMD_GET_HEALTH = [0xA5, 0x52];

  UsbPort? _port;
  StreamSubscription<Uint8List>? _rxSub;

  // (옵션) 포인트 스트림은 부하가 큽니다. 기본은 사용 안 함.
  final _pointCtrl = StreamController<RplidarPoint>.broadcast();
  Stream<RplidarPoint> get points => _pointCtrl.stream;

  // 권장: 프레임(원시 배열) 스트림
  final _frameCtrl = StreamController<RplidarFrame>.broadcast();
  Stream<RplidarFrame> get framesRaw => _frameCtrl.stream;

  // Parser isolate
  Isolate? _parserIso;
  SendPort? _toParser;
  StreamSubscription? _fromParserSub;

  Future<UsbDevice?> _pickDevice() async {
    final devices = await UsbSerial.listDevices();
    devices.sort((a, b) {
      int score(UsbDevice d) => (d.vid == 0x10c4) ? 0 : 2; // CP210x 우선
      return score(a).compareTo(score(b));
    });
    return devices.isNotEmpty ? devices.first : null;
  }

  Future<void> open() async {
    final dev = await _pickDevice();
    if (dev == null) {
      throw StateError('USB 장치를 찾을 수 없습니다. OTG 연결 및 권한을 확인하세요.');
    }
    final port = await dev.create();
    if (port == null || !await port.open()) {
      throw StateError('포트를 열 수 없습니다.');
    }
    await port.setDTR(true);
    await port.setRTS(true);
    await port.setPortParameters(
      baud,
      UsbPort.DATABITS_8,
      UsbPort.STOPBITS_1,
      UsbPort.PARITY_NONE,
    );
    _port = port;

    // 파서 Isolate 기동
    await _spawnParserIsolate();

    // USB 수신 → 파서로 TransferableTypedData로 전달(복사 최소화)
    _rxSub = _port!.inputStream!.listen(
          (chunk) {
        final ttd = TransferableTypedData.fromList([chunk]);
        _toParser?.send(_Msg.chunkTtd(ttd));
      },
      onError: (e, st) => debugPrint('RPLIDAR RX error: $e'),
      cancelOnError: false,
    );
  }

  Future<void> close() async {
    await stopScan();
    await _rxSub?.cancel();
    await _port?.close();
    _rxSub = null;
    _port = null;

    _toParser?.send(const _Msg.close());
    _toParser = null;
    await _fromParserSub?.cancel();
    _fromParserSub = null;
    _parserIso?.kill(priority: Isolate.immediate);
    _parserIso = null;

    await _pointCtrl.close();
    await _frameCtrl.close();
  }

  Future<void> _write(List<int> data) async {
    final p = _port;
    if (p == null) throw StateError('포트가 열려있지 않습니다.');
    await p.write(Uint8List.fromList(data));
  }

  Future<void> getInfoAndHealth() async {
    await _write(_CMD_GET_INFO);
    await Future.delayed(const Duration(milliseconds: 10));
    await _write(_CMD_GET_HEALTH);
  }

  Future<void> startScan() async {
    await _write(_CMD_STOP);
    await Future.delayed(const Duration(milliseconds: 5));
    _toParser?.send(const _Msg.reset());
    await _write(_CMD_SCAN);
  }

  Future<void> stopScan() async {
    if (_port != null) {
      await _write(_CMD_STOP);
      await Future.delayed(const Duration(milliseconds: 5));
    }
  }

  // ──────────────────────────────────────────────
  // Isolate wiring
  // ─────────────────────────────────────────────-
  Future<void> _spawnParserIsolate() async {
    final fromParser = ReceivePort();
    _parserIso = await Isolate.spawn<_IsoBoot>(
      _parserMain,
      _IsoBoot(fromParser.sendPort),
      debugName: 'rplidar_parser',
    );

    _fromParserSub = fromParser.listen((msg) {
      if (msg is SendPort) {
        _toParser = msg;
        return;
      }
      if (msg is _FrameMsg) {
        // ★ 원시 배열 그대로 UI로 전달(객체 생성 제거)
        _frameCtrl.add(RplidarFrame(msg.angles, msg.dists, msg.quals));

        // (선택) 레거시 포인트 이벤트도 내보내려면 아래 주석 해제 (부하 큼)
        // for (int i = 0; i < msg.angles.length; i++) {
        //   _pointCtrl.add(RplidarPoint(
        //     msg.angles[i].toDouble(),
        //     msg.dists[i].toDouble(),
        //     msg.quals[i],
        //     false,
        //   ));
        // }
        return;
      }
    });
  }
}

/// ──────────────────────────────────────────────
/// Isolate message types
/// ─────────────────────────────────────────────-
class _IsoBoot {
  final SendPort toMain;
  const _IsoBoot(this.toMain);
}

class _Msg {
  final int kind; // 0=chunk,1=reset,2=close
  final TransferableTypedData? ttd;
  final Uint8List? bytes;
  const _Msg._(this.kind, {this.ttd, this.bytes});
  const _Msg.reset() : this._(1);
  const _Msg.close() : this._(2);
  static _Msg chunk(Uint8List c) => _Msg._(0, bytes: c);
  static _Msg chunkTtd(TransferableTypedData t) => _Msg._(0, ttd: t);
}

class _FrameMsg {
  final Float32List angles;
  final Float32List dists;
  final Uint8List quals;
  int get len => angles.length;
  _FrameMsg(this.angles, this.dists, this.quals);
}

/// ──────────────────────────────────────────────
/// Parser Isolate
/// ─────────────────────────────────────────────-
void _parserMain(_IsoBoot boot) {
  final toMain = boot.toMain;
  final fromMain = ReceivePort();
  toMain.send(fromMain.sendPort);

  // 링 버퍼
  Uint8List bb = Uint8List(8192);
  int r = 0, w = 0;
  void ensureCap(int need) {
    if (bb.length >= need) return;
    var n = bb.length;
    while (n < need) n <<= 1;
    final nb = Uint8List(n);
    nb.setRange(0, w - r, bb.sublist(r, w));
    w = w - r;
    r = 0;
    bb = nb;
  }

  bool descriptorSkipped = false;
  bool seenStart = false;
  final curFrame = <_Pt>[];

  const int maxRangeMm = 12000;

  // (옵션) 출력 디메이트: 1이면 전체, 2면 절반, 3이면 1/3…
  const int DECIMATE = 2;

  void finalizeAndSendFrame() {
    final n = curFrame.length;
    if (n == 0) return;

    final inc = 360.0 / n;

    // head tune
    int i = 0;
    for (; i < n; i++) {
      if (curFrame[i].distMm > 0) {
        for (int j = i - 1; j >= 0; j--) {
          final expect = _wrap360(curFrame[j + 1].angleDeg - inc);
          curFrame[j] = curFrame[j].copyWithAngle(expect);
        }
        break;
      }
    }
    if (i == n) {
      _emitFrame(curFrame, toMain, maxRangeMm, decimate: DECIMATE);
      return;
    }

    // tail tune
    i = n - 1;
    for (; i >= 0; i--) {
      if (curFrame[i].distMm > 0) {
        for (int j = i + 1; j < n; j++) {
          var expect = curFrame[j - 1].angleDeg + inc;
          if (expect >= 360.0) expect -= 360.0;
          curFrame[j] = curFrame[j].copyWithAngle(expect);
        }
        break;
      }
    }

    // invalid 각도 채우기
    final frontAngle = curFrame[0].angleDeg;
    for (int k = 1; k < n; k++) {
      if (curFrame[k].distMm <= 0) {
        var expect = frontAngle + k * inc;
        if (expect >= 360.0) expect -= 360.0;
        curFrame[k] = curFrame[k].copyWithAngle(expect);
      }
    }

    _emitFrame(curFrame, toMain, maxRangeMm, decimate: DECIMATE);
  }

  void reset() {
    descriptorSkipped = false;
    seenStart = false;
    curFrame.clear();
    r = 0; w = 0;
  }

  fromMain.listen((raw) {
    if (raw is! _Msg) return;
    switch (raw.kind) {
      case 1: // reset
        reset();
        return;
      case 2: // close
        fromMain.close();
        return;
      case 0: // chunk
        final Uint8List chunk = raw.ttd != null
            ? raw.ttd!.materialize().asUint8List()
            : (raw.bytes ?? Uint8List(0));

        ensureCap(w + chunk.length);
        bb.setRange(w, w + chunk.length, chunk);
        w += chunk.length;

        int i = r;

        // 디스크립터 7바이트 1회 스킵
        if (!descriptorSkipped && i + 7 <= w) {
          if (bb[i] == 0xA5 && bb[i + 1] == 0x5A) {
            i += 7;
            descriptorSkipped = true;
          }
        }

        // 5바이트 샘플 파싱
        while (i + 5 <= w) {
          final b0 = bb[i], b1 = bb[i + 1], b2 = bb[i + 2], b3 = bb[i + 3], b4 = bb[i + 4];

          final s   = (b0 & 0x01);
          final nS  = (b0 >> 1) & 0x01;
          final okS = ((s ^ nS) == 1);
          final cOk = (b1 & 0x01) != 0;
          if (!(okS && cOk)) { i += 1; continue; }

          final angleQ6  = ((b1 >> 1) | (b2 << 7)) & 0x7FFF;
          final angleDeg = angleQ6 / 64.0;
          final distQ2   = (b3) | (b4 << 8);
          final distMm   = distQ2 / 4.0;
          final qual6    = (b0 >> 2) & 0x3F;
          final start    = s != 0;

          if (start && seenStart && curFrame.isNotEmpty) {
            finalizeAndSendFrame();
            curFrame.clear();
          }
          curFrame.add(_Pt(angleDeg, distMm, qual6));
          if (start) seenStart = true;

          i += 5;
        }

        r = i;
        // 압축(드물게)
        if (r > 4096 && (w - r) < (bb.length >> 1)) {
          final remain = bb.sublist(r, w);
          bb.setRange(0, remain.length, remain);
          w = remain.length;
          r = 0;
        }
        return;
    }
  });
}

// 내부 포인트 (isolate 전용)
class _Pt {
  final double angleDeg;
  final double distMm;
  final int qual;
  _Pt(this.angleDeg, this.distMm, this.qual);
  _Pt copyWithAngle(double a) => _Pt(a, distMm, qual);
}

double _wrap360(double a) {
  var x = a % 360.0;
  if (x < 0) x += 360.0;
  return x;
}

void _emitFrame(List<_Pt> src, SendPort toMain, int maxRangeMm, {int decimate = 1}) {
  // 필터 + (옵션) 디메이트
  final n0 = src.length;
  if (n0 == 0) return;

  // 대략적 upper bound(필터/디메이트 후 실제 n은 이보다 작을 수 있음)
  final angles = Float32List((n0 / decimate).ceil());
  final dists  = Float32List((n0 / decimate).ceil());
  final quals  = Uint8List((n0 / decimate).ceil());

  int j = 0;
  for (int i = 0; i < n0; i += decimate) {
    final p = src[i];
    if (p.distMm <= 0 || p.distMm >= maxRangeMm) continue;
    angles[j] = p.angleDeg.toDouble();
    dists[j]  = p.distMm.toDouble();
    quals[j]  = p.qual;
    j++;
  }

  if (j == 0) return;

  // 실제 길이만 잘라서 보냄
  toMain.send(_FrameMsg(
    angles.sublist(0, j),
    dists.sublist(0, j),
    quals.sublist(0, j),
  ));
}
