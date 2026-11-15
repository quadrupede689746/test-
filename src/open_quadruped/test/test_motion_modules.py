import math
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parents[1] / 'nodes'))
from gait_generator import swing_legs, foot_trajectory
from balance import stability_margin
from safety_monitor import SafetyMonitor, SafetyState


def test_crawl_never_has_more_than_one_swing_and_repeats():
    n=1000
    samples=[swing_legs(i*24/n,24) for i in range(n)]
    assert max(map(len,samples)) <= 1
    assert samples[0] == swing_legs(24,24)


def test_trajectory_is_continuous_at_cycle_boundary():
    a=foot_trajectory('bl', 1e-7, (1,0), .005, .03, 24)
    b=foot_trajectory('bl', 24-1e-7, (1,0), .005, .03, 24)
    assert max(abs(x-y) for x,y in zip(a,b)) < 1e-6


def test_square_support_margin_hand_calculation():
    assert math.isclose(stability_margin([(-1,-1),(1,-1),(1,1),(-1,1)],(0,0)),1.)
    assert math.isclose(stability_margin([(-1,-1),(1,-1),(1,1),(-1,1)],(1.2,0)),-.2)


def test_safety_states():
    monitor=SafetyMonitor()
    assert monitor.evaluate(contacts=dict(fl=True,fr=True,bl=True,br=True)) is SafetyState.OK
    assert monitor.evaluate(roll=.3) is SafetyState.WARNING
    assert monitor.evaluate(pitch=.7) is SafetyState.FALL
    assert monitor.evaluate(contacts=dict(fl=True,fr=True,bl=False,br=False)) is SafetyState.WARNING
    assert monitor.evaluate(contacts=dict(fl=True,fr=False,bl=False,br=False)) is SafetyState.FALL


def test_urdf_fk_ik_round_trip_and_joint_limits():
    import pytest
    try:
        from kinematics import UrdfLegKinematics
        model=UrdfLegKinematics()
    except Exception as exc:
        pytest.skip(f'ROS package index or xacro unavailable: {exc}')
    for leg, (_, names) in model.chains.items():
        q=[0.,-.68162889,1.26606748]
        target=model.fk(leg,q)
        result=model.ik(leg,target,q)
        assert math.dist(target,model.fk(leg,result)) < .001
        assert all(model.limits[name][0] <= angle <= model.limits[name][1]
                   for name,angle in zip(names,result))
