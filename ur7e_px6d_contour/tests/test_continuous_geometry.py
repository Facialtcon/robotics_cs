"""Integrated execution + independent geometric force model, not scripted F/T."""
import numpy as np
import pytest
from simulation.continuous_validation import run_case


@pytest.mark.parametrize('scene',['track_straight','track_circle'])
def test_contact_tracking_from_search(scene):
    report, trajectory, _=run_case(scene)
    assert 'FIRST_CONTACT' in report['events']
    assert report['final_state']=='CONTINUOUS_TRACKING'
    assert report['tracking_contact_fraction'] > .95
    assert np.linalg.norm(trajectory[-1]-trajectory[0]) > .005
    assert report['max_force_N'] < 2.25
    assert report['max_penetration_m'] == 0


@pytest.mark.parametrize('scene',['recover_straight','recover_arc','recover_endpoint'])
def test_actual_arc_path_reacquires_geometric_contact(scene):
    report, trajectory, reference=run_case(scene)
    assert report['recovery_success'], report
    assert np.linalg.norm(trajectory[-1]-trajectory[0]) > .0003
    assert report['max_tracking_error_m'] < .0005
    assert report['recovery_path_length_m'] < .0035
    assert report['max_force_N'] < 2.25
    assert report['max_penetration_m']==0
    assert np.isfinite(reference).any()


def test_no_target_in_range_is_explicit_bounded_failure():
    report, trajectory, _=run_case('recover_empty')
    assert not report['recovery_success']
    assert report['final_state']=='STOP'
    assert 'angle' in report['stop_reason'] or 'time' in report['stop_reason']
    assert report['max_force_N']==0
    assert report['recovery_path_length_m'] < .0035
    assert np.max(np.linalg.norm(trajectory-trajectory[0],axis=1)) < .0035


@pytest.mark.parametrize('rotation,mirror,hand',[(37,False,'COUNTERCLOCKWISE'),(179,False,'COUNTERCLOCKWISE'),
                                              (0,True,'CLOCKWISE'),(37,True,'CLOCKWISE')])
def test_recovery_geometry_rotates_and_mirrors_consistently(rotation,mirror,hand):
    base, path, _=run_case('recover_endpoint')
    report, transformed,_=run_case('recover_endpoint',rotation_deg=rotation,mirror=mirror,hand=hand)
    assert report['recovery_success'] == base['recovery_success']
    theta=np.deg2rad(rotation)
    matrix=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])@np.diag([1,-1 if mirror else 1])
    assert np.allclose(transformed,path@matrix.T,atol=1e-9)


def test_extreme_actuation_lag_reconfirms_with_independent_geometric_audit():
    report,_,_=run_case('track_endpoint',delay_steps=30)
    assert report['final_state']=='CONTINUOUS_TRACKING'
    assert 'DIRECTION_CONFIRMED' in report['events']
    assert 'DIRECTION_RESUME_VERIFIED' in report['events']
    assert report['actual_path_length_m'] > .005
    assert report['max_penetration_m'] < 1e-9
    assert 'REACQUIRED' not in report['events']
    assert report['max_force_N'] < 2.25
