"""robot_config の設定ファイルが読めて, 計算した値が矛盾しないことを確かめる (pytest)."""
import math

import pytest

from robot_config import load, robots


@pytest.mark.parametrize('name', robots())
def test_every_robot_loads(name):
    cfg = load(name)
    for node in cfg.node_names():
        assert isinstance(cfg.node(node), dict)
    nav2 = cfg.nav2_params()
    for costmap in ('local_costmap', 'global_costmap'):
        p = nav2[costmap][costmap]['ros__parameters']
        assert p['footprint'].startswith('[[')
        assert p['use_sim_time'] == cfg.sim


@pytest.mark.parametrize('name', robots())
def test_mount_and_inverse_cancel(name):
    cfg = load(name)
    (t, rpy), (ti, rpyi) = cfg.mount_tf(), cfg.inverse_mount_tf()
    # 取付 (base_link -> LiDAR) と逆 (LiDAR -> base_link) を続けると元に戻る
    from robot_config.config import rotation_rpy
    r, ri = rotation_rpy(*rpy), rotation_rpy(*rpyi)
    p = [sum(r[i][j] * ti[j] for j in range(3)) + t[i] for i in range(3)]
    assert all(abs(v) < 1e-9 for v in p)
    prod = [[sum(r[i][k] * ri[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    assert all(abs(prod[i][j] - (1.0 if i == j else 0.0)) < 1e-9 for i in range(3) for j in range(3))


def test_shared_values_are_the_same():
    cfg = load('rc_car')
    bridge, det, rmap = cfg.node('lidar_bridge'), cfg.node('obstacle_detector'), cfg.node('robot_map')
    for k in ('lidar_x', 'lidar_pitch_deg', 'self_box_min', 'self_box_max'):
        assert bridge[k] == det[k]
    for k in ('ground_z', 'obstacle_min_height', 'obstacle_max_height'):
        assert rmap[k] == det[k]
    calib = cfg.node('calib_web')
    assert calib['wheelbase'] == cfg.node('cmd_vel_converter')['wheelbase']
    assert math.isclose(calib['lidar_x'], 0.25)
    assert calib['port'] == cfg.node('launcher')['mode_port']
