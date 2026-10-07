"""走行キャリブレーションの解析 (ROS に依存しない部分).

calibrate_drive が記録した raw.csv / segments.csv から
  * スロットル(PWM) -> 速度 [m/s]
  * サーボ角 [deg]  -> 前輪の舵角 [deg]
の対応を求める。

座標系
  オドメトリは Point-LIO の camera_init -> aft_mapped (LiDAR/IMU 位置)。
  gravity_align: true なので camera_init の z は鉛直。LiDAR は前に 30 度
  傾いているが X 軸が前を向いているので、機体 X 軸を水平面に投影した向きを
  ヨー角として使う (ピッチの影響を受けない)。

舵角の求め方 (自転車モデル)
  後輪軸中心の速度 v_r, ヨーレート w, ホイールベース L とすると
    tan(delta) = L * w / v_r = L / R_r
  LiDAR は後輪軸から前に d [m] ずれているので、LiDAR の速さ v_l は
    v_l^2 = v_r^2 + (w * d)^2,  R_l^2 = R_r^2 + d^2
  の関係から後輪軸の値に直す。
"""
import csv
import math
import os

import numpy as np

RAW_FIELDS = ['t', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw']
SEGMENT_FIELDS = ['id', 'kind', 'servo', 'throttle', 't_start', 't_end', 'end_reason']


# --------------------------------------------------------------------------
# 基本計算
# --------------------------------------------------------------------------
def yaw_of_body_x(qx, qy, qz, qw):
    """機体 X 軸をワールド水平面に投影した向き [rad] (ピッチが大きくても安定)."""
    # R * (1, 0, 0) の x, y 成分
    xx = 1.0 - 2.0 * (qy * qy + qz * qz)
    xy = 2.0 * (qx * qy + qw * qz)
    return math.atan2(xy, xx)


def fit_circle(xs, ys):
    """最小二乗 (Kasa 法) で円をあてはめる. (cx, cy, R, rms) を返す."""
    xs = np.asarray(xs, dtype=float)
    ys = np.asarray(ys, dtype=float)
    a = np.column_stack([xs, ys, np.ones_like(xs)])
    b = xs * xs + ys * ys
    sol, *_ = np.linalg.lstsq(a, b, rcond=None)
    cx, cy = sol[0] / 2.0, sol[1] / 2.0
    r = math.sqrt(max(sol[2] + cx * cx + cy * cy, 0.0))
    rms = float(np.sqrt(np.mean((np.hypot(xs - cx, ys - cy) - r) ** 2)))
    return cx, cy, r, rms


def path_length(xs, ys, min_step=0.03):
    """ノイズで伸びないよう、min_step [m] 以上離れた点だけをつないだ道のり."""
    total = 0.0
    px, py = xs[0], ys[0]
    for x, y in zip(xs[1:], ys[1:]):
        step = math.hypot(x - px, y - py)
        if step >= min_step:
            total += step
            px, py = x, y
    total += math.hypot(xs[-1] - px, ys[-1] - py)
    return total


def slope(t, v):
    """v = a * t + b の a."""
    t = np.asarray(t, dtype=float)
    v = np.asarray(v, dtype=float)
    return float(np.polyfit(t - t[0], v, 1)[0])


# --------------------------------------------------------------------------
# 区間ごとの解析
# --------------------------------------------------------------------------
def analyze_straight(samples):
    """直進区間: 進行方向に沿った移動量の傾きを速度とする (後退は負)."""
    t = samples[:, 0]
    x, y = samples[:, 1], samples[:, 2]
    yaws = np.unwrap(samples[:, 3])
    heading = math.atan2(np.mean(np.sin(yaws)), np.mean(np.cos(yaws)))
    hx, hy = math.cos(heading), math.sin(heading)
    along = (x - x[0]) * hx + (y - y[0]) * hy
    lateral = -(x - x[0]) * hy + (y - y[0]) * hx
    return {
        'speed': slope(t, along),
        'yaw_rate': slope(t, yaws),
        'lateral_drift': float(lateral[-1]),
        'duration': float(t[-1] - t[0]),
        'distance': float(along[-1]),
    }


def analyze_turn(samples, wheelbase, lidar_x, min_arc_deg=90.0):
    """旋回区間: 曲率 -> 後輪軸の旋回半径 -> 舵角."""
    t = samples[:, 0]
    x, y = samples[:, 1], samples[:, 2]
    yaws = np.unwrap(samples[:, 3])
    duration = float(t[-1] - t[0])
    yaw_rate = slope(t, yaws)
    arc = float(yaws[-1] - yaws[0])

    # 前進 / 後退の符号は、進行方向への移動量で判定
    dx, dy = np.diff(x), np.diff(y)
    along = dx * np.cos(yaws[:-1]) + dy * np.sin(yaws[:-1])
    direction = 1.0 if np.sum(along) >= 0.0 else -1.0
    v_lidar = direction * path_length(x, y) / duration if duration > 0 else 0.0

    result = {
        'speed_lidar': v_lidar,
        'yaw_rate': yaw_rate,
        'arc_deg': math.degrees(arc),
        'duration': duration,
        'circle_radius_lidar': float('nan'),
        'circle_rms': float('nan'),
        'source': 'rate',
    }

    # 1) 速度とヨーレートから (どんな短い弧でも使える)
    v_rear = direction * math.sqrt(max(v_lidar ** 2 - (yaw_rate * lidar_x) ** 2, 0.0))
    curvature = yaw_rate / v_rear if abs(v_rear) > 1e-3 else float('nan')

    # 2) 十分に回っていれば、円のあてはめのほうがノイズに強い
    if abs(math.degrees(arc)) >= min_arc_deg:
        _, _, r_lidar, rms = fit_circle(x, y)
        result['circle_radius_lidar'] = r_lidar
        result['circle_rms'] = rms
        r_rear = math.sqrt(max(r_lidar ** 2 - lidar_x ** 2, 1e-9))
        # 左旋回 (前進で yaw 増加) を正にする
        curvature = math.copysign(1.0 / r_rear, yaw_rate * direction)
        result['source'] = 'circle'

    result['curvature'] = curvature
    result['radius_rear'] = 1.0 / curvature if abs(curvature) > 1e-6 else float('inf')
    if wheelbase and wheelbase > 0 and not math.isnan(curvature):
        result['steer_deg'] = math.degrees(math.atan(wheelbase * curvature))
    else:
        result['steer_deg'] = float('nan')
    return result


# --------------------------------------------------------------------------
# 全体のフィット
# --------------------------------------------------------------------------
def fit_throttle(rows, min_speed=0.03):
    """speed = gain * (throttle - deadband) を前進/後退それぞれでフィット."""
    out = {}
    for name, sign in (('forward', 1), ('backward', -1)):
        pts = [(r['throttle'], r['speed']) for r in rows
               if r['throttle'] * sign > 0 and r['speed'] * sign > min_speed]
        if len(pts) < 2:
            out[name] = None
            continue
        thr = np.array([p[0] for p in pts], dtype=float)
        spd = np.array([p[1] for p in pts], dtype=float)
        gain, offset = np.polyfit(thr, spd, 1)
        stalled = [r['throttle'] for r in rows
                   if r['throttle'] * sign > 0 and r['speed'] * sign <= min_speed]
        out[name] = {
            'gain_mps_per_throttle': float(gain),
            'deadband_throttle': float(-offset / gain) if gain else float('nan'),
            'max_stalled_throttle': max(stalled, key=abs) if stalled else None,
            'points': len(pts),
        }
    return out


def fit_steering(rows):
    """サーボ角 -> 舵角. 線形 delta = k (phi - phi0) と 正弦 sin(delta) = K sin(phi - phi0)."""
    pts = [(r['servo'], r['steer_deg']) for r in rows if not math.isnan(r['steer_deg'])]
    if len(pts) < 2:
        return None
    phi = np.array([p[0] for p in pts], dtype=float)
    delta = np.array([p[1] for p in pts], dtype=float)
    k, b = np.polyfit(phi, delta, 1)
    phi0 = -b / k if k else float('nan')
    s_phi = np.sin(np.radians(phi - phi0))
    s_delta = np.sin(np.radians(delta))
    big_k = float(np.dot(s_phi, s_delta) / np.dot(s_phi, s_phi))
    resid_lin = delta - (k * phi + b)
    return {
        'linear_ratio': float(k),               # d(delta)/d(phi)
        'servo_center_deg': float(phi0),        # 舵角 0 になるサーボ角
        'sine_ratio_K': big_k,                  # r / l
        'linear_rms_deg': float(np.sqrt(np.mean(resid_lin ** 2))),
        'left_max_deg': float(delta.max()),
        'right_max_deg': float(delta.min()),
        'points': len(pts),
    }


# --------------------------------------------------------------------------
# ファイル入出力
# --------------------------------------------------------------------------
def load_run(out_dir):
    with open(os.path.join(out_dir, 'raw.csv')) as f:
        raw = np.array([[float(r[k]) for k in RAW_FIELDS] for r in csv.DictReader(f)])
    with open(os.path.join(out_dir, 'segments.csv')) as f:
        segments = list(csv.DictReader(f))
    for s in segments:
        s['servo'] = int(s['servo'])
        s['throttle'] = int(s['throttle'])
        s['t_start'] = float(s['t_start'])
        s['t_end'] = float(s['t_end'])
    return raw, segments


def to_xyyaw(raw):
    yaw = [yaw_of_body_x(*q) for q in raw[:, 4:8]]
    return np.column_stack([raw[:, 0], raw[:, 1], raw[:, 2], yaw])


def analyze_run(out_dir, wheelbase, lidar_x, settle=1.0, min_samples=10):
    """記録済みの run を解析し, CSV と summary.yaml を out_dir に書き出す."""
    raw, segments = load_run(out_dir)
    data = to_xyyaw(raw)
    throttle_rows, steering_rows = [], []

    for s in segments:
        mask = (data[:, 0] >= s['t_start'] + settle) & (data[:, 0] <= s['t_end'])
        samples = data[mask]
        if len(samples) < min_samples:
            print(f"[WARN] segment {s['id']} ({s['kind']}): samples too few ({len(samples)})")
            continue
        if s['kind'] == 'throttle':
            r = analyze_straight(samples)
            r.update(servo=s['servo'], throttle=s['throttle'], id=s['id'])
            throttle_rows.append(r)
        elif s['kind'] == 'steering':
            r = analyze_turn(samples, wheelbase, lidar_x)
            r.update(servo=s['servo'], throttle=s['throttle'], id=s['id'])
            steering_rows.append(r)

    throttle_rows.sort(key=lambda r: r['throttle'])
    steering_rows.sort(key=lambda r: r['servo'])
    _write_rows(os.path.join(out_dir, 'throttle_result.csv'), throttle_rows,
                ['id', 'throttle', 'servo', 'speed', 'yaw_rate', 'lateral_drift',
                 'distance', 'duration'])
    _write_rows(os.path.join(out_dir, 'steering_result.csv'), steering_rows,
                ['id', 'servo', 'throttle', 'steer_deg', 'curvature', 'radius_rear',
                 'speed_lidar', 'yaw_rate', 'arc_deg', 'circle_radius_lidar',
                 'circle_rms', 'source', 'duration'])

    summary = {
        'wheelbase': wheelbase,
        'lidar_x_from_rear_axle': lidar_x,
        'throttle': fit_throttle(throttle_rows),
        'steering': fit_steering(steering_rows),
    }
    _write_yaml(os.path.join(out_dir, 'summary.yaml'), summary)
    _print_report(throttle_rows, steering_rows, summary)
    return summary


def _write_rows(path, rows, fields):
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=fields, extrasaction='ignore')
        w.writeheader()
        for r in rows:
            w.writerow({k: (round(v, 5) if isinstance(v, float) else v)
                        for k, v in r.items()})


def _write_yaml(path, data, indent=0):
    """PyYAML に頼らない簡易 YAML 出力 (dict / None / 数値のみ)."""
    lines = []

    def emit(d, level):
        for k, v in d.items():
            pad = '  ' * level
            if isinstance(v, dict):
                lines.append(f'{pad}{k}:')
                emit(v, level + 1)
            elif v is None:
                lines.append(f'{pad}{k}: null')
            elif isinstance(v, float):
                lines.append(f'{pad}{k}: {v:.5g}')
            else:
                lines.append(f'{pad}{k}: {v}')

    emit(data, indent)
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def _print_report(throttle_rows, steering_rows, summary):
    if throttle_rows:
        print('\n=== スロットル -> 速度 ===')
        print(f"{'throttle':>8} {'speed[m/s]':>11} {'yaw_rate':>9} {'drift[m]':>9}")
        for r in throttle_rows:
            print(f"{r['throttle']:>8d} {r['speed']:>11.3f} {r['yaw_rate']:>9.3f} "
                  f"{r['lateral_drift']:>9.3f}")
        for name, fit in summary['throttle'].items():
            if fit:
                print(f"  {name}: speed = {fit['gain_mps_per_throttle']:.4f} * "
                      f"(throttle {-fit['deadband_throttle']:+.1f})")
    if steering_rows:
        print('\n=== サーボ角 -> 舵角 (左が正) ===')
        print(f"{'servo':>6} {'steer[deg]':>11} {'R_rear[m]':>10} {'arc[deg]':>9} {'src':>7}")
        for r in steering_rows:
            print(f"{r['servo']:>6d} {r['steer_deg']:>11.2f} {r['radius_rear']:>10.2f} "
                  f"{r['arc_deg']:>9.1f} {r['source']:>7}")
        fit = summary['steering']
        if fit:
            print(f"  linear: steer = {fit['linear_ratio']:.3f} * "
                  f"(servo - {fit['servo_center_deg']:.1f})   rms {fit['linear_rms_deg']:.2f} deg")
            print(f"  sine  : sin(steer) = {fit['sine_ratio_K']:.3f} * "
                  f"sin(servo - {fit['servo_center_deg']:.1f})")
