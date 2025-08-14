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
  StreamSubscription<RplidarPoint>? sub;

  /// 화면에 실제로 그릴 리스트(동일 인스턴스 유지; 내용만 교체)
  final points = <RplidarPoint>[];

  /// 최신 데이터(리스너가 갱신) + 버전
  List<RplidarPoint> _latestPoints = const [];
  int _dataVersion = 0;
  int _paintedVersion = -1;

  Timer? _repaintTimer;
  bool scanning = false;

  // 성능용: 각도 룩업테이블(0.5° 스텝)
  late final _AngleLut _lut = _AngleLut(0.5);

  // 반경 매퍼(1~4 m 압축 비율)
  static const _mapper = _RadiusMapper(
    maxRangeMm: 4000,
    kneeStartMm: 1000,  // 1 m
    kneeEndMm: 4000,    // 4 m
    innerFrac: 0.60,    // 1~4 m가 차지할 화면 반경 비율(원하면 0.80으로)
  );

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
    final frames = <List<RplidarPoint>>[];
    List<RplidarPoint> currentFrame = [];
    bool seenStart = false;

    points.clear();
    _latestPoints = const [];
    _dataVersion = 0;
    _paintedVersion = -1;

    await sub?.cancel();

    // 스트림: 여기서는 setState 하지 않음 — 최신 데이터만 보관
    sub = lidar.points.listen((p) {
      if (p.startFlag) {
        if (seenStart && currentFrame.isNotEmpty) {
          frames.add(List<RplidarPoint>.from(currentFrame));
          if (frames.length > 1) frames.removeAt(0); // 최근 1회전만 유지(원하면 N으로)

          _latestPoints = frames.expand((f) => f).toList(growable: false);
          _dataVersion++;
          currentFrame.clear();
        }
        seenStart = true;
      }

      if (p.angleDeg >= 140 && p.angleDeg <= 300) return;
      currentFrame.add(p);
    });

    // 30fps 타이머: 데이터 버전 바뀌었을 때만 페인트
    _repaintTimer?.cancel();
    _repaintTimer = Timer.periodic(const Duration(milliseconds: 100), (_) {
      if (_dataVersion != _paintedVersion) {
        setState(() {
          points
            ..clear()
            ..addAll(_latestPoints);
          _paintedVersion = _dataVersion;
        });
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

          /// 레이어 분리: Stack + RepaintBoundary 두 장
          Expanded(
            child: Container(
              color: Colors.black,
              child: Stack(
                fit: StackFit.expand,
                children: [
                  /// 고정 레이어(그리드/스포크/라벨) — 거의 다시 그리지 않음
                  RepaintBoundary(
                    child: CustomPaint(
                      painter: _GridPainter(_mapper),
                      child: const SizedBox.expand(),
                    ),
                  ),
                  RepaintBoundary(
                    child: CustomPaint(
                      painter: _PointsPainter(points, _lut, _mapper,_dataVersion),
                      child: const SizedBox.expand(),
                    ),
                  ),
                  /// 동적 레이어(포인트) — 30fps 타이머로만 갱신
                ],
              ),
            ),
          ),
        ],
      ),
    );
  }
}

/// ─────────────────────────────────────────────────────────────────────────
/// 각도 LUT (성능: sin/cos 매 호출 방지)
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

  /// 0°=위, 반시계, 좌우 반전 포함 → 방향 벡터 얻기
  /// x = -sin(rad), y = -cos(rad)
  void dir(double angleDeg, List<double> out2) {
    int idx = (angleDeg / stepDeg).round() % len;
    if (idx < 0) idx += len;
    out2[0] = -sinLut[idx];
    out2[1] = -cosLut[idx];
  }
}

/// ─────────────────────────────────────────────────────────────────────────
/// 비선형 반경 매핑 (1~4m: innerFrac, 4~12m: 나머지)
class _RadiusMapper {
  final int maxRangeMm;   // 12000
  final int kneeStartMm;  // 1000 (1 m)
  final int kneeEndMm;    // 4000 (4 m)
  final double innerFrac; // 1~4 m가 차지할 반경 비율

  const _RadiusMapper({
    this.maxRangeMm = 12000,
    this.kneeStartMm = 1000,
    this.kneeEndMm = 4000,
    this.innerFrac = 0.60,
  });

  double mapMmToPx(int dMm, double totalR) {
    if (dMm <= 0) return 0;
    if (dMm >= maxRangeMm) return totalR;

    final r1 = totalR * innerFrac;  // 1~4 m
    final r2 = totalR - r1;         // 4~12 m

    if (dMm <= kneeStartMm) {
      final t0 = dMm / kneeStartMm;                 // 0~1 m
      return t0 * (r1 * (kneeStartMm / kneeEndMm)); // 1 m 지점까지 선형
    } else if (dMm <= kneeEndMm) {
      final t1 = (dMm - kneeStartMm) / (kneeEndMm - kneeStartMm);
      final rAt1m = (r1 * (kneeStartMm / kneeEndMm));
      return rAt1m + t1 * (r1 - rAt1m);
    } else {
      final t2 = (dMm - kneeEndMm) / (maxRangeMm - kneeEndMm);
      return r1 + t2 * r2;
    }
  }
}

/// ─────────────────────────────────────────────────────────────────────────
/// 고정 배경: 그리드/스포크/라벨
class _GridPainter extends CustomPainter {
  final _RadiusMapper mapper;
  const _GridPainter(this.mapper);

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

    // 중심 십자
    canvas.drawLine(Offset(center.dx, 0), Offset(center.dx, size.height), axis);
    canvas.drawLine(Offset(0, center.dy), Offset(size.width, center.dy), axis);

    // // 반경 그리드(2 m 간격)
    // for (int rMm = 2000; rMm <= mapper.maxRangeMm; rMm += 2000) {
    //   final rPx = mapper.mapMmToPx(rMm, totalR);
    //   final p = (rMm % 4000 == 0) ? gridBold : grid;
    //   canvas.drawCircle(center, rPx, p);
    //
    //   // 라벨(오른쪽)
    //   _drawLabel(canvas,
    //       offset: Offset(center.dx + rPx, center.dy),
    //       text: '${(rMm / 1000).toStringAsFixed(0)}m',
    //       alignCenter: false);
    // }

    // 각도 스포크 & 라벨 — 0°=위, CCW, 좌우 반전 포함
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
  bool shouldRepaint(covariant _GridPainter oldDelegate) => false; // 고정
}

/// ─────────────────────────────────────────────────────────────────────────
/// 동적 전경: 포인트(점) — 15fps(66ms) 타이머로 갱신
class _PointsPainter extends CustomPainter {
  final List<RplidarPoint> pts;
  final _AngleLut lut;
  final _RadiusMapper mapper; // maxRangeMm만 사용
  final int version; // 데이터 버전으로 리페인트 트리거

  _PointsPainter(this.pts, this.lut, this.mapper, this.version);

  // 재사용 버퍼
  final _dir = List<double>.filled(2, 0.0, growable: false);

  @override
  void paint(Canvas canvas, Size size) {
    if (pts.isEmpty) return;

    final center   = Offset(size.width / 2, size.height / 2);
    final longest  = math.max(size.width, size.height);
    final shortest = math.min(size.width, size.height);

    // ★ 포인트 전용 매핑 반경
    final double rOuter = longest  * 0.45; // 1~12 m가 도달하는 바깥 반경(긴 변 기준)
    final double rInner = shortest * 0.45; // 0~1 m가 도달하는 내부 반경(짧은 변 기준)

    const int innerMaxMm = 1000; // 1 m
    final int maxMm = mapper.maxRangeMm; // 12000

    // 좌표를 한 번에 준비 → drawRawPoints 1회 호출
    final coords = Float32List(pts.length * 2);
    int j = 0;

    for (final p in pts) {
      final d = p.distanceMm.toInt();
      if (d <= 0 || d > maxMm) continue;

      // ★ 새로운 매핑: 0..1m -> 0..rInner, 1..12m -> rInner..rOuter
      double r;
      if (d <= innerMaxMm) {
        final t0 = d / innerMaxMm;        // 0..1
        r = t0 * rInner;
      } else {
        final t1 = (d - innerMaxMm) / (maxMm - innerMaxMm); // 0..1
        r = rInner + t1 * (rOuter - rInner);
      }

      // 0°=위, CCW, 좌우 반전
      lut.dir(p.angleDeg, _dir);
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

    // 실제 채워진 좌표만 그리기
    canvas.drawRawPoints(PointMode.points, coords.sublist(0, j), dot);
  }

  @override
  bool shouldRepaint(covariant _PointsPainter old) => old.version != version;
}
