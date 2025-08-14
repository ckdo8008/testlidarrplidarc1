import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';

import 'package:flutter/cupertino.dart';
import 'package:usb_serial/usb_serial.dart';
// import 'package:usb_serial/transaction.dart'; // 미사용

class RplidarPoint {
  final double angleDeg;   // 0..360
  final double distanceMm; // mm
  final int quality;       // 0..255
  final bool startFlag;    // 회전 시작(S=1)
  RplidarPoint(this.angleDeg, this.distanceMm, this.quality, this.startFlag);
}

class RplidarC1 {
  static const int baud = 460800; // C1 권장 보드레이트
  static const List<int> _CMD_SCAN = [0xA5, 0x20];
  static const List<int> _CMD_STOP = [0xA5, 0x25];
  static const List<int> _CMD_GET_INFO = [0xA5, 0x50];
  static const List<int> _CMD_GET_HEALTH = [0xA5, 0x52];

  UsbPort? _port;
  StreamSubscription<Uint8List>? _rxSub;
  final _pointCtrl = StreamController<RplidarPoint>.broadcast();
  Stream<RplidarPoint> get points => _pointCtrl.stream;

  // ─────────────────────────────────────────────────────────────────────────────
  // 연결/설정
  // ─────────────────────────────────────────────────────────────────────────────
  Future<UsbDevice?> _pickDevice() async {
    final devices = await UsbSerial.listDevices();
    // CP210x(0x10C4) 우선
    devices.sort((a, b) {
      int score(UsbDevice d) {
        if (d.vid == 0x10c4) return 0;
        return 2;
      }
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
    if (port != null) {
      if (!await port.open()) {
        throw StateError('포트를 열 수 없습니다.');
      }
    }

    // 460800, 8N1
    await port?.setDTR(true);
    await port?.setRTS(true);
    await port?.setPortParameters(
      baud,
      UsbPort.DATABITS_8,
      UsbPort.STOPBITS_1,
      UsbPort.PARITY_NONE,
    );
    _port = port;

    _rxSub = _port!.inputStream!.listen(
      _onBytes,
      onError: (e, st) {
        debugPrint('RPLIDAR RX error: $e');
      },
      cancelOnError: false,
    );
  }

  Future<void> close() async {
    await stopScan();
    await _rxSub?.cancel();
    await _port?.close();
    _rxSub = null;
    _port = null;
  }

  Future<void> _write(List<int> data) async {
    if (_port == null) throw StateError('포트가 열려있지 않습니다.');
    await _port!.write(Uint8List.fromList(data));
  }

  Future<void> getInfoAndHealth() async {
    await _write(_CMD_GET_INFO);
    await Future.delayed(const Duration(milliseconds: 10));
    await _write(_CMD_GET_HEALTH);
  }

  Future<void> startScan() async {
    await _write(_CMD_STOP);
    await Future.delayed(const Duration(milliseconds: 5));
    _legacyDescriptorSkipped = false; // 디스크립터 스킵 플래그 리셋
    _seenStart = false;
    _curFrame.clear();
    await _write(_CMD_SCAN);
  }

  Future<void> stopScan() async {
    if (_port != null) {
      await _write(_CMD_STOP);
      await Future.delayed(const Duration(milliseconds: 5));
    }
  }

  // ─────────────────────────────────────────────────────────────────────────────
  // 레거시 SCAN 파서(5 bytes per sample) + 프레임 보정/정렬
  // ─────────────────────────────────────────────────────────────────────────────
  final _buf = BytesBuilder();
  bool _legacyDescriptorSkipped = false;

  // 프레임 버퍼 (한 바퀴)
  final List<RplidarPoint> _curFrame = <RplidarPoint>[];
  bool _seenStart = false;

  void _onBytes(Uint8List chunk) {
    _buf.add(chunk);
    final data = _buf.toBytes();
    int i = 0;

    // (1) 응답 디스크립터는 시작 구간에서 '한 번만' 스킵
    if (!_legacyDescriptorSkipped && i + 7 <= data.length) {
      if (data[i] == 0xA5 && data[i + 1] == 0x5A) {
        // A5 5A 05 00 00 40 81
        i += 7;
        _legacyDescriptorSkipped = true;
      }
    }

    // (2) 5바이트 측정 패킷 파싱
    final out = <RplidarPoint>[];
    while (i + 5 <= data.length) {
      final b0 = data[i], b1 = data[i + 1], b2 = data[i + 2], b3 = data[i + 3], b4 = data[i + 4];

      final s = (b0 & 0x01);
      final notS = (b0 >> 1) & 0x01;
      final okS = ((s ^ notS) == 1);

      final cOk = (b1 & 0x01) != 0; // C bit == 1 (byte1 LSB)
      if (!(okS && cOk)) { i += 1; continue; }

      // angle_q6 조립: (b1 >> 1) = angle_q6[6:0], (b2 << 7) = angle_q6[14:7]
      final angleQ6 = ((b1 >> 1) | (b2 << 7)) & 0x7FFF; // ★ 수정: 0x7FFF
      final angleDeg = angleQ6 / 64.0;

      final distQ2 = (b3) | (b4 << 8);
      final distanceMm = distQ2 / 4.0;

      final qual6 = (b0 >> 2) & 0x3F;

      out.add(RplidarPoint(angleDeg, distanceMm, qual6, s != 0));
      i += 5;
    }

    // (3) 남은 바이트 보관
    if (i > 0) {
      final remain = Uint8List.fromList(data.sublist(i));
      _buf.clear();
      _buf.add(remain);
    }

    // (4) 프레임 구성 및 각도 보정/정렬 후 방출
    for (final p in out) {
      if (p.startFlag && _seenStart && _curFrame.isNotEmpty) {
        // 이전 프레임 종료 → SDK ascendScanData_ 적용
        final fixed = _ascendScanData(_curFrame);
        // distance>0 만 방출
        for (final fp in fixed) {
          if (fp.distanceMm > 0 && fp.distanceMm < 12000) {
            _pointCtrl.add(fp);
          }
        }
        _curFrame.clear();
      }
      _curFrame.add(p);
      if (p.startFlag) _seenStart = true;
    }
  }

  int _indexOfSeq(Uint8List hay, List<int> needle) {
    for (int k = 0; k + needle.length <= hay.length; k++) {
      bool ok = true;
      for (int j = 0; j < needle.length; j++) {
        if (hay[k + j] != needle[j]) { ok = false; break; }
      }
      if (ok) return k;
    }
    return -1;
  }

  // SDK ascendScanData_ 로직을 Dart로 재현 (각도 채움 + 정렬)
  List<RplidarPoint> _ascendScanData(List<RplidarPoint> src) {
    final n = src.length;
    if (n == 0) return const <RplidarPoint>[];

    // 깊은 복사(각도를 수정해야 함)
    final nodes = src
        .map((e) => RplidarPoint(e.angleDeg, e.distanceMm, e.quality, e.startFlag))
        .toList(growable: false);

    final inc = 360.0 / n;

    // Head tune: 첫 유효거리 샘플을 찾고, 그 이전 각도를 되감아 채우기
    int i = 0;
    for (; i < n; i++) {
      if (nodes[i].distanceMm > 0) {
        for (int j = i - 1; j >= 0; j--) {
          final expect = _wrap360(nodes[j + 1].angleDeg - inc);
          nodes[j] = RplidarPoint(expect, nodes[j].distanceMm, nodes[j].quality, nodes[j].startFlag);
        }
        break;
      }
    }
    if (i == n) {
      // 전부 distance==0 → 원본 반환
      return nodes;
    }

    // Tail tune: 마지막 유효거리 샘플 이후 각도를 채우기
    i = n - 1;
    for (; i >= 0; i--) {
      if (nodes[i].distanceMm > 0) {
        for (int j = i + 1; j < n; j++) {
          var expect = nodes[j - 1].angleDeg + inc;
          if (expect >= 360.0) expect -= 360.0;
          nodes[j] = RplidarPoint(expect, nodes[j].distanceMm, nodes[j].quality, nodes[j].startFlag);
        }
        break;
      }
    }

    // Invalid angle(거리==0)에 대해서도 각도 값을 메우기
    final frontAngle = nodes[0].angleDeg;
    for (int k = 1; k < n; k++) {
      if (nodes[k].distanceMm <= 0) {
        var expect = frontAngle + k * inc;
        if (expect >= 360.0) expect -= 360.0;
        nodes[k] = RplidarPoint(expect, nodes[k].distanceMm, nodes[k].quality, nodes[k].startFlag);
      }
    }

    // 각도 정렬
    nodes.sort((a, b) => a.angleDeg.compareTo(b.angleDeg));
    return nodes;
  }

  double _wrap360(double a) {
    var x = a % 360.0;
    if (x < 0) x += 360.0;
    return x;
  }
}
