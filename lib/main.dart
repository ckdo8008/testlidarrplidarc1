import 'dart:async';
import 'dart:math' as math;
import 'dart:typed_data';
import 'dart:ui';
import 'package:flutter/material.dart';
import 'rplidar_c1.dart';

void main() {
  runApp(const MaterialApp(home: LidarPage()));
}

class LidarPage extends StatefulWidget {
  const LidarPage({super.key});
  @override
  State<LidarPage> createState() => _LidarPageState();
}

class _LidarPageState extends State<LidarPage> {
  final lidar = RplidarC1();
  StreamSubscription<RplidarFrame>? sub;

  // 최신 프레임(원시 배열)
  RplidarFrame? _latest;
  int _dataVersion = 0;       // 프레임이 들어올 때마다 증가
  int _preparedDataVer = -1;  // 좌표가 준비된 데이터 버전
  int _paintedVersion = -1;   // 페인터 트리거용

  // 캔버스 사이즈 트래킹
  Size? _canvasSize;
  int _sizeStamp = 0;          // 사이즈 변경 시 증가
  int _preparedSizeStamp = -1; // 좌표가 준비된 사이즈 스탬프

  // 미리 산출된 좌표 버퍼
  Float32List _coords = Float32List(0);       // 내부 작업용(충분히 큰 버퍼)
  Float32List _coordsForDraw = Float32List(0); // 그릴 길이만 보이는 view
  int _coordsLen = 0;                         // 좌표 개수*2

  Timer? _repaintTimer;
  bool scanning = false;

  // 각도 LUT
  // late final _AngleLut _lut = _AngleLut(0.5);
  late final _AngleLut _lut = _AngleLut(1.0);

  // 매핑 파라미터
  static const int _innerMaxMm = 1000;   // 0~1m는 짧은 변 기준
  static const int _maxMm = 6000;       // 1~6m는 긴 변 기준

  @override
  void dispose() {
    _repaintTimer?.cancel();
    sub?.cancel();
    lidar.close();
    super.dispose();
  }

  Future<void> _connect() async {
    await lidar.open();
    await lidar.getInfoAndHealth();
    ScaffoldMessenger.of(context).showSnackBar(
      const SnackBar(content: Text('RPLIDAR 연결됨')),
    );
  }

  Future<void> _start() async {
    _latest = null;
    _dataVersion = 0;
    _preparedDataVer = -1;
    _preparedSizeStamp = -1;
    _paintedVersion = -1;
    await sub?.cancel();

    // 프레임(원시 배열) 구독: 객체 생성 없음
    sub = lidar.framesRaw.listen((frame) {
      _latest = frame;
      _dataVersion++;
      // 사이즈가 이미 있으면 곧바로 좌표 준비(빌드/타이머 기다릴 필요 최소화)
      if (_canvasSize != null) {
        _prepareCoordsIfNeeded();
      }
    });

    // 리페인트 타이머(저부하 15fps)
    _repaintTimer?.cancel();
    _repaintTimer = Timer.periodic(const Duration(milliseconds: 66), (_) {
      _prepareCoordsIfNeeded(); // 필요할 때만 setState 발생
    });

    await lidar.startScan();
    setState(() => scanning = true);
  }

  Future<void> _stop() async {
    await lidar.stopScan();
    await sub?.cancel();
    _repaintTimer?.cancel();
    setState(() => scanning = false);
  }

  // 좌표 사전 산출기 (데이터/사이즈 변경 때만 수행)
  void _prepareCoordsIfNeeded() {
    final f = _latest;
    final sz = _canvasSize;
    if (f == null || sz == null) return;

    // 변경 없으면 스킵
    if (_preparedDataVer == _dataVersion && _preparedSizeStamp == _sizeStamp) {
      return;
    }

    final centerDx = sz.width  / 2.0;
    final centerDy = sz.height / 2.0;
    final longest  = math.max(sz.width, sz.height);
    final shortest = math.min(sz.width, sz.height);
    final double rOuter = longest  * 0.45; // 1..12 m
    final double rInner = shortest * 0.45; // 0..1 m

    // 필요시 버퍼 증설(2 * 점수)
    final need = f.length * 2;
    if (_coords.length < need) {
      _coords = Float32List(need);
    }

    int j = 0;
    final angles = f.angles;
    final dists  = f.dists;
    final dir = _tmpDir; // 재사용 버퍼

    for (int i = 0; i < f.length; i++) {
      final d = dists[i].toInt();
      if (d <= 0 || d > _maxMm) continue;

      double r;
      if (d <= _innerMaxMm) {
        final t0 = d / _innerMaxMm;                 // 0..1
        r = t0 * rInner;
      } else {
        final t1 = (d - _innerMaxMm) / (_maxMm - _innerMaxMm); // 0..1
        r = rInner + t1 * (rOuter - rInner);
      }

      // 0°=위, CCW, 좌우 반전
      _lut.dir(angles[i], dir);
      if (dir[0] == 0.0 && dir[1] == 0.0) continue;
      _coords[j++] = centerDx + r * dir[0];
      _coords[j++] = centerDy + r * dir[1];
    }

    _coordsLen = j;

    // 그릴 길이만 보이는 view를 준비 시점에 미리 만들어둔다(페인트에서 추가 할당 없음)
    _coordsForDraw = (_coordsLen == 0)
        ? Float32List(0)
        : Float32List.view(_coords.buffer, 0, _coordsLen);

    // 준비된 버전/사이즈 기록
    _preparedDataVer = _dataVersion;
    _preparedSizeStamp = _sizeStamp;

    // painter 트리거(최소 setState)
    setState(() {
      _paintedVersion++;
    });
  }

  // 재사용 임시 버퍼(각도->단위벡터)
  final List<double> _tmpDir = List<double>.filled(2, 0.0, growable: false);

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      appBar: AppBar(title: const Text('RPLIDAR C1 (Flutter/Android)')),
      body: Column(
        children: [
          Row(
            children: [
              ElevatedButton(onPressed: _connect, child: const Text('연결')),
              const SizedBox(width: 8),
              ElevatedButton(
                onPressed: scanning ? null : _start,
                child: const Text('스캔 시작'),
              ),
              const SizedBox(width: 8),
              ElevatedButton(
                onPressed: scanning ? _stop : null,
                child: const Text('중지'),
              ),
            ],
          ),
          const SizedBox(height: 8),

          // ★ 사이즈를 캡처하기 위해 LayoutBuilder 사용
          Expanded(
            child: LayoutBuilder(
              builder: (context, constraints) {
                final newSize = Size(constraints.maxWidth, constraints.maxHeight);
                if (_canvasSize == null || _canvasSize != newSize) {
                  _canvasSize = newSize;
                  _sizeStamp++;
                  // 사이즈 바뀌면 다음 프레임에 좌표 준비
                  WidgetsBinding.instance.addPostFrameCallback((_) {
                    _prepareCoordsIfNeeded();
                  });
                }

                return Container(
                  color: Colors.black,
                  child: Stack(
                    fit: StackFit.expand,
                    children: [
                      RepaintBoundary(
                        child: CustomPaint(
                          painter: const _GridPainter(),
                          child: const SizedBox.expand(),
                        ),
                      ),
                      RepaintBoundary(
                        child: CustomPaint(
                          // ★ 좌표만 넘겨서 그리기 — painter는 drawRawPoints 한 번만 호출
                          painter: _PointsPainterPrecomputed(
                            _coordsForDraw,
                            _paintedVersion,
                          ),
                          child: const SizedBox.expand(),
                        ),
                      ),
                    ],
                  ),
                );
              },
            ),
          ),
        ],
      ),
    );
  }
}

/// 각도 LUT
class _AngleLut {
  final double stepDeg;
  late final int len;
  late final Float32List cosLut;
  late final Float32List sinLut;

  _AngleLut([this.stepDeg = 0.5]) {
    len = (360.0 / stepDeg).round();
    cosLut = Float32List(len);
    sinLut = Float32List(len);
    for (int i = 0; i < len; i++) {
      final deg = i * stepDeg;
      if (deg >= 140 && deg <= 300) {
        cosLut[i] = 0;
        sinLut[i] = 0;
        continue;
      }

      final rad = deg * math.pi / 180.0;
      cosLut[i] = math.cos(rad);
      sinLut[i] = math.sin(rad);
    }
  }

  /// 0°=위, CCW, 좌우 반전 포함 → 방향 벡터
  void dir(double angleDeg, List<double> out2) {
    int idx = (angleDeg / stepDeg).round() % len;
    if (idx < 0) idx += len;
    out2[0] = -sinLut[idx];
    out2[1] = -cosLut[idx];
  }
}

/// 배경 그리드(고정)
class _GridPainter extends CustomPainter {
  const _GridPainter();
  @override
  void paint(Canvas canvas, Size size) {
    final center = Offset(size.width / 2, size.height / 2);
    final shortest = math.min(size.width, size.height);
    final totalR = shortest * 0.45;

    final grid = Paint()
      ..color = const Color(0xFF333333)
      ..strokeWidth = 1
      ..isAntiAlias = false;
    final gridBold = Paint()
      ..color = const Color(0xFF3E3E3E)
      ..strokeWidth = 1.2
      ..isAntiAlias = false;
    final axis = Paint()
      ..color = const Color(0xFF444444)
      ..strokeWidth = 1
      ..isAntiAlias = false;

    canvas.drawLine(Offset(center.dx, 0), Offset(center.dx, size.height), axis);
    canvas.drawLine(Offset(0, center.dy), Offset(size.width, center.dy), axis);

    // 스포크(0°=위)
    for (int deg = 0; deg < 360; deg += 10) {
      final rad = deg * math.pi / 180.0;
      final dir = Offset(-math.sin(rad), -math.cos(rad));
      final rOuter = totalR;
      final isMajor = deg % 30 == 0;

      final rInner = isMajor ? rOuter * 0.85 : rOuter * 0.9;
      canvas.drawLine(center + dir * rInner, center + dir * rOuter,
          isMajor ? gridBold : grid);

      if (isMajor) {
        final labelPos = center + dir * (rOuter + 12);
        _drawLabel(canvas, offset: labelPos, text: '$deg°');
      }
    }
  }

  static void _drawLabel(Canvas canvas,
      {required Offset offset, required String text, bool alignCenter = true}) {
    final tp = TextPainter(
      text: TextSpan(
        text: text,
        style: const TextStyle(color: Color(0xFF9E9E9E), fontSize: 10),
      ),
      textAlign: TextAlign.center,
      textDirection: TextDirection.ltr,
    )..layout();
    final dx = alignCenter ? offset.dx - tp.width / 2 : offset.dx + 4;
    final dy = offset.dy - tp.height / 2;
    tp.paint(canvas, Offset(dx, dy));
  }

  @override
  bool shouldRepaint(covariant _GridPainter oldDelegate) => false;
}

/// 전경 포인트(미리 산출된 좌표만 그리기)
class _PointsPainterPrecomputed extends CustomPainter {
  final Float32List coordsForDraw; // 길이 j짜리 view
  final int version;               // 그리기 버전(변경 시에만 repaint)
  _PointsPainterPrecomputed(this.coordsForDraw, this.version);

  @override
  void paint(Canvas canvas, Size size) {
    if (coordsForDraw.isEmpty) return;

    final dot = Paint()
      ..color = const Color(0xFF00FFAA)
      ..strokeWidth = 2
      ..isAntiAlias = false;

    // 루프 없음 — 한 번에 그리기
    canvas.drawRawPoints(PointMode.points, coordsForDraw, dot);
  }

  @override
  bool shouldRepaint(covariant _PointsPainterPrecomputed old) =>
      !identical(old.coordsForDraw, coordsForDraw) || old.version != version;
}
