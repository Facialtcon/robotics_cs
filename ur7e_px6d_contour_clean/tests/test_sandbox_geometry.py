"""All-corner rectangle fitting and exact inset geometry; independent containment audits."""
from copy import deepcopy

import numpy as np
import pytest
import yaml

from app.sandbox_scan import ROOT, make_plan
from calibration.sandbox_scan import load_record, save_record
from workspace.sandbox_scan import plan_coverage
from workspace.workspace_calibrator import fit_workspace, effective_scan_rectangle
from workspace.workspace_transform import WorkspaceTransform, load_calibration, validate_calibration
from test_sandbox_scan import rectangle, record_for


def corners(fitted):
    return np.array([fitted['rectified_points'][f'R{i}'][:2] for i in range(4)])


def edge_clearances(raw, points):
    """Independent point-to-line distances, preserving the measured edge order."""
    area = sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(raw,np.roll(raw,-1,axis=0)))
    result = []
    for a,b in zip(raw,np.roll(raw,-1,axis=0)):
        edge = b-a
        delta = np.asarray(points)-a
        result.append(np.sign(area)*(edge[0]*delta[:,1]-edge[1]*delta[:,0])/np.linalg.norm(edge))
    return np.array(result).T


@pytest.mark.parametrize('angle', [0.,31.,89.,133.])
@pytest.mark.parametrize('reverse', [False,True])
def test_global_fit_uses_all_corners_preserving_order_and_winding(angle,reverse):
    measured=rectangle(angle,noise=.006,reverse=reverse)
    fitted=fit_workspace(measured['raw_points'],method='least_squares')
    raw=np.array([[measured['raw_points'][f'P{i}'][a] for a in 'xy'] for i in range(4)])
    points=corners(fitted)
    assert fitted['raw_points']==measured['raw_points']
    assert fitted['winding']==measured['winding']
    np.testing.assert_allclose(points.mean(axis=0),raw.mean(axis=0),atol=1e-14)
    edges=np.roll(points,-1,axis=0)-points
    for i in range(4):
        assert edges[i] @ edges[(i+1)%4]==pytest.approx(0.,abs=1e-14)
    assert fitted['diagnostics']['fit_rms_xy_m'] <= measured['diagnostics']['fit_rms_xy_m']+1e-14
    # Relabeling the same perimeter cannot privilege a new first edge.
    for order in ([1,2,3,0],[3,2,1,0],[2,3,0,1]):
        shifted=fit_workspace({f'P{i}':measured['raw_points'][f'P{j}'] for i,j in enumerate(order)},method='least_squares')
        np.testing.assert_allclose(corners(shifted),points[order],atol=1e-13)
    # Independently solve center/half-lengths at many fixed orientations using
    # linear least squares on all eight coordinate observations.
    sx,sy=np.array([-1,1,1,-1]),np.array([-1,-1,1,1])
    best=float(np.sum((points-raw)**2))
    for theta in np.linspace(0,2*np.pi,721):
        x=np.array([np.cos(theta),np.sin(theta)])
        y=fitted['normal_sign']*np.array([-x[1],x[0]])
        design=np.zeros((8,4))
        design[::2,0]=design[1::2,1]=1
        design[:,2]=(sx[:,None]*x).ravel()
        design[:,3]=(sy[:,None]*y).ravel()
        solution=np.linalg.lstsq(design,raw.ravel(),rcond=None)[0]
        if np.all(solution[2:]>0):
            assert best <= np.sum((design @ solution-raw.ravel())**2)+1e-13


@pytest.mark.parametrize('angle', [0.,37.,112.])
@pytest.mark.parametrize('reverse', [False,True])
def test_exact_rotated_rectangle_keeps_exact_margin_without_fit_padding(angle,reverse):
    measured=rectangle(angle,reverse=reverse)
    fitted=fit_workspace(measured['raw_points'],method='least_squares')
    region=effective_scan_rectangle(fitted,.004)
    np.testing.assert_allclose(corners(fitted),
        [[measured['raw_points'][f'P{i}'][a] for a in 'xy'] for i in range(4)],atol=1e-13)
    np.testing.assert_allclose(region['local_min_xy'],[.004,.004],atol=1e-13)
    np.testing.assert_allclose(region['local_max_xy'],np.array([fitted['length'],fitted['width']])-.004,atol=1e-13)
    clearance=edge_clearances(np.array(region['raw_corners_xy']),region['effective_corners_xy'])
    assert np.min(clearance)==pytest.approx(.004,abs=1e-13)


@pytest.mark.parametrize('angle', [0.,41.,127.])
def test_skew_polygon_contains_effective_region_scan_and_connectors(angle,config):
    xy=np.array([[0.,0.],[.24,0.],[.29,-.64],[.04,-.642]])
    theta=np.deg2rad(angle);rotation=np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]])
    xy=xy @ rotation.T + [.4,.1]
    raw={f'P{i}':np.r_[point,i*.2,0.,0.,0.] for i,point in enumerate(xy)}
    fitted=fit_workspace(raw,method='least_squares')
    region=effective_scan_rectangle(fitted,.0037125)
    center=corners(fitted).mean(axis=0)
    np.testing.assert_allclose(np.mean(region['effective_corners_xy'],axis=0),center,atol=1e-13)
    clearance=edge_clearances(xy,region['effective_corners_xy'])
    assert np.min(clearance)>=.0037125-1e-13
    assert np.min(clearance)==pytest.approx(.0037125,abs=1e-12)
    for p0 in (xy[0],center,xy[2]+[.1,.1]):
        plan=plan_coverage(region,np.r_[p0,3.,0.,0.,0.],scan_z=-.1,line_spacing=.017)
        route=np.array(plan['coverage_xy'])
        samples=np.vstack([a+(b-a)*np.linspace(0,1,51)[:,None] for a,b in zip(route,route[1:])])
        assert np.min(edge_clearances(xy,samples))>=.0037125-1e-13
        assert plan['actual_line_spacing_m']<=.017
        assert plan['scan_z']==-.1


def test_current_saved_raw_calibration_recomputes_fit_and_inside_region_without_writes(config):
    from app.configuration import search_stopping_margin
    path=ROOT/'calibration_data/sandbox_scan.yaml'
    before=path.read_bytes()
    record=load_record(path)
    plan=make_plan(config,record)
    geometry=plan['geometry']; fitted=geometry['fitted_workspace']
    legacy=fit_workspace(record['workspace']['raw_points'])
    assert fitted['diagnostics']['fit_rms_xy_m'] <= legacy['diagnostics']['fit_rms_xy_m']+1e-13
    assert plan['boundary_margin_m']==pytest.approx(search_stopping_margin(config))
    assert np.min(edge_clearances(np.array(plan['raw_corners_xy']),plan['effective_corners_xy'])) >= plan['boundary_margin_m']-1e-13
    assert path.read_bytes()==before
    assert set(record['workspace'])=={'raw_points','metadata'}


def test_calibration_only_persists_raw_points_and_ignores_stale_derived_data(config,tmp_path,monkeypatch):
    from workspace import workspace_calibrator
    record=record_for(config,size=.025)
    expected=deepcopy(record['workspace']['raw_points'])
    record['workspace']['length']='stale old fit'
    record['workspace']['rectified_points']=None
    path=tmp_path/'raw.yaml'
    path.write_text(yaml.safe_dump(dict(sandbox_scan=record)))
    before=path.read_bytes()
    with monkeypatch.context() as patch:
        def forbidden(*args,**kwargs):
            raise AssertionError('calibration I/O must not fit geometry')
        patch.setattr(workspace_calibrator,'fit_workspace',forbidden)
        loaded=load_record(path)
        save_record(tmp_path/'new.yaml',loaded)
    assert path.read_bytes()==before
    stored=yaml.safe_load((tmp_path/'new.yaml').read_text())['sandbox_scan']
    assert stored['workspace']==dict(raw_points=expected,metadata={})
    plan=make_plan(config,loaded)
    assert len(plan['rectified_corners_xy'])==4
    assert plan['geometry']['fitted_workspace']['fit_method']=='least_squares'


def test_legacy_fit_still_loads_and_new_fit_roundtrips_with_explicit_method(tmp_path):
    legacy=load_calibration(ROOT/'workspace/config/workspace_calibration.yaml')
    assert 'fit_method' not in legacy
    assert fit_workspace(legacy['raw_points'],metadata=legacy['metadata'])==legacy
    new=fit_workspace(legacy['raw_points'],method='least_squares')
    assert validate_calibration(new)==new
    from workspace.workspace_calibrator import save_calibration
    path=tmp_path/'derived.yaml';save_calibration(new,path)
    assert load_calibration(path)==new
    transform=WorkspaceTransform(new)
    points=np.array([[.01,.02,.03],[.1,.03,-.1]])
    np.testing.assert_allclose(transform.base_to_workspace(transform.workspace_to_base(points)),points,atol=1e-13)
