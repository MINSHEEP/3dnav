"""CPU profile isolation and bounded parameter transactions, without ROS."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from car_vel.nav2_timing_profile import BaselineStore, TimingProfilePolicy, scoped_profile


def test_only_xingji_terrain_3d_selects_cpu_profile():
    assert scoped_profile('terrain_3d', 'xingji') == 'xingji_3d'
    assert scoped_profile('map_2d', 'xingji') == 'baseline'
    assert scoped_profile('terrain_3d', 'gangben_zsl1') == 'baseline'
    assert scoped_profile('map_2d', 'gangben_zsl1') == 'baseline'
    assert scoped_profile('terrain_3d', '') == 'unknown'


def test_profile_halves_candidate_work_without_changing_controller_geometry():
    policy = TimingProfilePolicy()
    policy.set_context('terrain_3d', 'xingji', active=False)
    assert policy.next_operation() == ('read', None)
    policy.observe_actual(800)
    assert policy.next_operation() == ('set', 400)
    assert policy.status()['ready'] is False
    policy.begin('set', 400, now=1.0)
    policy.finish_set(True)
    assert policy.next_operation() == ('read', None)
    policy.observe_actual(400)
    assert policy.status()['ready'] is True
    assert policy.next_operation() is None
    assert 400 * 25 == (800 * 25) // 2


def test_active_navigation_never_reconfigures_optimizer():
    policy = TimingProfilePolicy()
    policy.observe_actual(800)
    policy.set_context('terrain_3d', 'xingji', active=True)
    assert policy.next_operation() is None
    policy.set_context('terrain_3d', 'xingji', active=False)
    assert policy.next_operation() == ('set', 400)


def test_leaving_scope_restores_exact_runtime_baseline():
    policy = TimingProfilePolicy()
    policy.observe_actual(620)
    policy.set_context('terrain_3d', 'xingji', active=False)
    policy.observe_actual(400)
    policy.set_context('map_2d', 'xingji', active=False)
    assert policy.next_operation() == ('set', 620)
    policy.observe_actual(620)
    assert policy.status()['ready'] is True
    policy.set_context('terrain_3d', 'gangben_zsl1', active=False)
    assert policy.next_operation() is None


def test_no_write_on_unknown_identity_or_incomplete_context():
    policy = TimingProfilePolicy()
    policy.observe_actual(800)
    policy.set_context('terrain_3d', '', active=False)
    assert policy.next_operation() is None
    assert policy.status()['profile'] == 'unknown'
    policy.set_context('unknown', 'xingji', active=False)
    assert policy.next_operation() is None


def test_small_custom_batch_is_not_increased():
    policy = TimingProfilePolicy()
    policy.observe_actual(300)
    policy.set_context('terrain_3d', 'xingji', active=False)
    assert policy.next_operation() is None
    assert policy.status()['requested_batch_size'] == 300


def test_unowned_custom_400_baseline_is_not_guessed_to_be_an_old_optimization():
    policy = TimingProfilePolicy(configured_baseline=800)
    policy.observe_actual(400)
    for mode, body in (('map_2d','xingji'),('terrain_3d','xingji'),('terrain_3d','gangben_zsl1')):
        policy.set_context(mode,body,active=False)
        assert policy.baseline == 400
        assert policy.next_operation() is None
        assert policy.status()['ready']


def test_proven_new_controller_captures_new_actual_baseline_even_if_exactly_400():
    policy = TimingProfilePolicy(configured_baseline=800,recovered_baseline=620)
    policy.controller_restarted()
    policy.observe_actual(400)
    policy.set_context('map_2d','xingji',active=False)
    assert policy.baseline == 400
    assert policy.next_operation() is None
    assert policy.status()['ready']


def test_normal_full_backend_restart_can_apply_optimization_to_new_instance():
    policy = TimingProfilePolicy(configured_baseline=800,recovered_baseline=800)
    policy.controller_restarted()
    policy.observe_actual(800)
    policy.set_context('terrain_3d','xingji',active=False)
    assert policy.next_operation() == ('set',400)


def test_corrupt_or_mismatched_record_is_an_explicit_error(tmp_path):
    store = BaselineStore(tmp_path/'profile.json',800)
    store.path.write_text('{broken',encoding='utf-8')
    try:
        store.load_record()
        assert False
    except ValueError:
        pass
    store.save(800,pending_write=True,controller_gid='same-controller')
    for changed in (BaselineStore(store.path,800,300),BaselineStore(store.path,620,400)):
        try:
            changed.load_record()
            assert False, 'a launch change cannot erase a pending remote mutation'
        except ValueError:
            pass
    store.path.write_text('{}',encoding='utf-8')
    try:
        store.load_record()
        assert False
    except ValueError:
        pass


def test_expired_transaction_is_bounded_and_requests_actual_readback():
    policy = TimingProfilePolicy(transaction_timeout=3.0)
    policy.observe_actual(800)
    policy.set_context('terrain_3d', 'xingji', active=False)
    policy.begin('set', 400, now=10.0)
    assert policy.next_operation() is None
    assert not policy.expire(12.99)
    assert policy.expire(13.0)
    assert policy.status()['state'] == 'awaiting_late_ack'
    assert policy.status()['actual_batch_size'] is None
    assert policy.next_operation() is None
    policy.finish_set(True)
    assert policy.next_operation() == ('read', None)


def test_timed_out_write_keeps_ownership_until_late_ack_then_restores_new_scope():
    policy = TimingProfilePolicy(transaction_timeout=3.0)
    policy.observe_actual(800)
    policy.set_context('terrain_3d', 'xingji', active=False)
    policy.begin('set', 400, now=1.0)
    policy.expire(4.0)
    policy.set_context('map_2d', 'xingji', active=False)
    assert policy.next_operation() is None
    try:
        policy.observe_actual(800)
        assert False, 'an early baseline read cannot authorize a future late write'
    except RuntimeError:
        pass
    policy.finish_set(True)
    policy.observe_actual(400)
    assert not policy.status()['ready']
    assert policy.next_operation() == ('set', 800)
    policy.begin('set', 800, now=5.0)
    policy.finish_set(True)
    policy.observe_actual(800)
    assert policy.status()['ready']


def test_scope_change_during_ack_uses_new_target_not_stale_completion():
    policy = TimingProfilePolicy()
    policy.observe_actual(800)
    policy.set_context('terrain_3d', 'xingji', active=False)
    policy.begin('set', 400, now=0.0)
    policy.set_context('map_2d', 'xingji', active=False)
    policy.finish_set(True)
    policy.observe_actual(400)
    assert policy.status()['ready'] is False
    assert policy.next_operation() == ('set', 800)


def test_rejected_transaction_never_claims_profile_applied():
    policy = TimingProfilePolicy()
    policy.observe_actual(800)
    policy.set_context('terrain_3d', 'xingji', active=False)
    policy.begin('set', 400, now=1.0)
    policy.finish_set(False, 'parameter rejected')
    assert not policy.status()['ready']
    assert policy.status()['error'] == 'parameter rejected'


def test_pending_write_ownership_survives_node_restart(tmp_path):
    store=BaselineStore(tmp_path/'profile.json',800)
    store.save(620,pending_write=True,controller_gid='controller-one')
    record=BaselineStore(tmp_path/'profile.json',800).load_record()
    assert record['pending_write'] is True
    assert record['controller_gid']=='controller-one'
    assert record['baseline']==620
    policy=TimingProfilePolicy(configured_baseline=800,recovered_baseline=record['baseline'])
    policy.uncertain_write('old write pending')
    policy.set_context('map_2d','xingji',active=False)
    assert policy.next_operation() is None
    assert not policy.status()['ready']
    policy.controller_restarted()
    policy.observe_actual(800)
    assert policy.baseline==800
    assert policy.status()['ready']


def test_pending_before_send_cannot_be_released_by_one_early_baseline_read(tmp_path):
    store=BaselineStore(tmp_path/'profile.json',800)
    store.save(800,pending_write=True,controller_gid='same-controller')
    record=store.load_record()
    policy=TimingProfilePolicy(configured_baseline=800,recovered_baseline=record['baseline'])
    policy.uncertain_write('recovered pending')
    policy.set_context('map_2d','xingji',active=False)
    assert policy.next_operation() is None
    try:
        policy.observe_actual(800)
        assert False
    except RuntimeError:
        pass


def test_set_and_readback_share_one_three_second_transaction_budget():
    policy=TimingProfilePolicy(transaction_timeout=3.0)
    policy.observe_actual(800)
    policy.set_context('terrain_3d','xingji',active=False)
    policy.begin('set',400,now=1.0)
    policy.finish_set(True)
    policy.begin('read',None,now=3.5)
    assert policy.deadline==4.0
    assert policy.expire(4.01)


def test_failed_optimization_returns_baseline_without_extending_velocity_timeout():
    policy=TimingProfilePolicy()
    policy.observe_actual(800)
    policy.set_context('terrain_3d','xingji',active=False)
    policy.begin('set',400,now=1.0)
    policy.finish_set(False,'rejected')
    policy.observe_actual(800)
    assert policy.target==800
    assert policy.status()['ready']
    assert policy.status()['fallback']
    assert policy.status()['changes_command_timeout'] is False
