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
  int _dataVersion = 0;
  int _paintedVersion = -1;

  Timer? _repaintTimer;
  bool scanning = false;

  // 각도 LUT
  late final _AngleLut _lut = _AngleLut(0.5);

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
    _paintedVersion = -1;
    await sub?.cancel();

    // 프레임(원시 배열) 구독: 객체 생성 없음
    sub = lidar.framesRaw.listen((frame) {
      // (선택) 간단 각도 필터
      // 여기선 페인터에서 처리하도록 두고, 그대로 저장만
      _latest = frame;
      _dataVersion++;
    });

    // 리페인트 타이머(가벼운 10~20fps 권장)
    _repaintTimer?.cancel();
    _repaintTimer = Timer.periodic(const Duration(milliseconds: 66), (_) {
      if (_dataVersion != _paintedVersion) {
        setState(() => _paintedVersion = _dataVersion);
      }
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
          Expanded(
            child: Container(
              color: Colors.black,
              child: Stack(
                fit: StackFit.expand,
                children: [
                  RepaintBoundary(
                    child: CustomPaint(
                      painter: _GridPainter(),
                      child: const SizedBox.expand(),
                    ),
                  ),
                  RepaintBoundary(
                    child: CustomPaint(
                      painter: _PointsPainterRaw(_latest, _lut, _dataVersion),
                      child: const SizedBox.expand(),
                    ),
                  ),
                ],
              ),
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

/// 전경 포인트(원시 프레임으로 그리기)
class _PointsPainterRaw extends CustomPainter {
  final RplidarFrame? frame;
  final _AngleLut lut;
  final int version;
  _PointsPainterRaw(this.frame, this.lut, this.version);

  final _dir = List<double>.filled(2, 0.0, growable: false);

  @override
  void paint(Canvas canvas, Size size) {
    final f = frame;
    if (f == null || f.length == 0) return;

    final center   = Offset(size.width / 2, size.height / 2);
    final longest  = math.max(size.width, size.height);
    final shortest = math.min(size.width, size.height);

    // 0~1 m는 짧은 변 기준, 1~12 m는 긴 변 기준(모든 점을 표시할 필요 없음)
    final double rOuter = longest  * 0.45; // 1..12 m
    final double rInner = shortest * 0.45; // 0..1 m
    const int innerMaxMm = 1000;
    const int maxMm = 5000;

    final n = f.length;
    final coords = Float32List(n * 2);
    int j = 0;

    for (int i = 0; i < n; i++) {
      final angle = f.angles[i];
      final d     = f.dists[i].toInt();

      // (선택) 각도 필터(필요시)
      // if (angle >= 140 && angle <= 300) continue;

      if (d <= 0 || d > maxMm) continue;

      double r;
      if (d <= innerMaxMm) {
        final t0 = d / innerMaxMm;
        r = t0 * rInner;
      } else {
        final t1 = (d - innerMaxMm) / (maxMm - innerMaxMm);
        r = rInner + t1 * (rOuter - rInner);
      }

      // 방향 벡터
      lut.dir(angle, _dir);
      final x = center.dx + r * _dir[0];
      final y = center.dy + r * _dir[1];

      coords[j++] = x;
      coords[j++] = y;
    }

    if (j == 0) return;

    final dot = Paint()
      ..color = const Color(0xFF00FFAA)
      ..strokeWidth = 2
      ..isAntiAlias = false;

    canvas.drawRawPoints(PointMode.points, coords.sublist(0, j), dot);
  }

  @override
  bool shouldRepaint(covariant _PointsPainterRaw old) => old.version != version;
}
