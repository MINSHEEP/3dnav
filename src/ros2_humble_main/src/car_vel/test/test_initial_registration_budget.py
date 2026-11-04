"""Initial 3D optimization must not inherit the low-latency tracking budget."""
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[2] / 'registration/icp_registration'


def test_initial_registration_has_separate_budget_without_changing_legacy():
    cfg = yaml.safe_load((ROOT / 'config/icp.yaml').read_text(encoding='utf-8'))['/icp_registration']['ros__parameters']
    assert cfg['ndt_maximum_iterations'] == 6
    assert cfg['gicp_maximum_iterations'] == 6
    assert cfg['gicp_maximum_optimizer_iterations'] == 4
    assert cfg.get('initial_ndt_maximum_iterations', cfg['ndt_maximum_iterations']) >= 12
    assert cfg.get('initial_gicp_maximum_iterations', cfg['gicp_maximum_iterations']) >= 24
    assert cfg.get('initial_gicp_maximum_optimizer_iterations', cfg['gicp_maximum_optimizer_iterations']) >= 8
    assert cfg['maximum_rmse'] == .2
    assert cfg['minimum_overlap'] == .55
    assert cfg['maximum_vertical_correction'] == .35


def test_both_matchers_use_request_scoped_profile_and_initial_ceiling_is_explicit():
    source = (ROOT / 'src/icp_registration.cpp').read_text(encoding='utf-8')
    assert source.count('icp::refinementBudget(goal->robust_validation, goal->tracking,') == 2
    assert 'ndt.setMaximumIterations(budget.ndt)' in source
    assert 'gicp.setMaximumIterations(budget.gicp)' in source
    assert 'gicp.setMaximumOptimizerIterations(budget.optimizer)' in source
    assert 'robust_initial && icp::iterationCeiling' in source
    assert 'REGISTRATION_REQUEST map=' in source
    assert 'REGISTRATION_CANDIDATE index=' in source
    assert 'ctx.last_candidate_error = message' in source


def test_tracking_limits_are_separate_and_final_output_uses_verified_full_transform():
    cfg = yaml.safe_load((ROOT / 'config/icp.yaml').read_text(encoding='utf-8'))['/icp_registration']['ros__parameters']
    assert cfg['maximum_rmse']==.20 and cfg['maximum_vertical_correction']==.35
    assert cfg['tracking_maximum_rmse']==.22
    assert cfg['tracking_maximum_vertical_correction']==.45
    source=(ROOT/'src/icp_registration.cpp').read_text(encoding='utf-8')
    assert 'icp::validationLimits(goal->robust_validation, goal->tracking,' in source
    assert 'vertical_correction > limits.vertical' in source
    assert 'metrics.rmse > limits.rmse' in source
    assert 'transform, aggregate, fine_target, frames, pose, candidate_metrics, false)' in source
    assert 'valid.push_back({transform, pose, candidate_metrics})' in source
    assert 'planar, aggregate, fine_target, frames, pose, candidate_metrics, false)' not in source
    assert 'icp::poseOutput(rigid, refined_map_from_base(2, 3)' in source
