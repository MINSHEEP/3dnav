"""Motion trace diagnosis and rate limits; no ROS/network/robot I/O."""
import json
import ast
import threading
import time
from types import SimpleNamespace as NS
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from car_vel.motion_diagnostics import MotionTrace, arbiter_reason, body_reason


class MotionDiagnosisTests(unittest.TestCase):
    def arbiter(self, **changes):
        state=dict(navigation_active=True, source='nav', input_seen=True,
            input_age_sec=.01, command_timeout_sec=.3, input=[.2,0,0], output=[.1,0,0],
            lio_ready=True, navigation_ready=True, emergency=False,
            terrain_enabled=True, body_known=True, terrain_fresh=True,
            terrain_safe=True, evidence='current_anchored', body_id='xingji', speed_limit_mps=.1)
        state.update(changes)
        return arbiter_reason(state)

    def test_zero_input_is_not_misdiagnosed_as_body_or_tf_failure(self):
        self.assertEqual(self.arbiter(input=[0,0,0],output=[0,0,0]),'upstream_zero')
        self.assertEqual(self.arbiter(input_seen=False),'upstream_missing')
        self.assertEqual(self.arbiter(input_age_sec=.5),'upstream_stale')

    def test_protection_reasons_distinguish_geometry_evidence_and_authority(self):
        for changes,code in [({'emergency':True},'stop_latched'),
                ({'lio_ready':False},'lio_paused_or_stale'),
                ({'navigation_ready':False},'navigation_paused_or_stale'),
                ({'body_known':False},'body_identity_invalid'),
                ({'terrain_fresh':False},'terrain_status_stale'),
                ({'terrain_safe':False},'terrain_unsafe'),
                ({'evidence':'current'},'ground_evidence_unanchored'),
                ({'input':[-.1,0,0]},'direction_not_authorized'),
                ({'speed_limit_mps':0},'speed_limit_zero')]:
            with self.subTest(code=code):self.assertEqual(self.arbiter(**changes),code)

    def test_2d_skips_terrain_diagnosis_and_still_reports_limiting(self):
        self.assertEqual(self.arbiter(terrain_enabled=False,terrain_safe=False,
            body_known=False,input=[.1,0,0],output=[.1,0,0]),'forwarded')
        self.assertEqual(self.arbiter(),'limited')
        self.assertEqual(self.arbiter(source='teleop'),'other_source_owns_output')
        self.assertEqual(self.arbiter(source='zero',connector_active=True),'other_source_owns_output')

    def test_body_reason_does_not_claim_udp_send_means_physical_movement(self):
        base=dict(frozen=False, mapping_paused=False, remote_active=False,
            emergency=False, status_fresh=True, standing=True, gait_ready=True,
            armed=True, input_seen=True, input_age_sec=.01, input=[.1,0,.1],
            send_success=True, send_age_sec=.01)
        self.assertEqual(body_reason(base),'udp_dispatched_unconfirmed')
        for changes,code in [({'armed':False},'motion_not_armed'),
                ({'status_fresh':False},'body_status_stale'),
                ({'standing':False},'body_not_standing'),
                ({'gait_ready':False},'gait_feedback_invalid'),
                ({'remote_active':True},'remote_owns_body'),
                ({'send_success':False},'udp_send_failed'),
                ({'send_success':False,'send_age_sec':1},'udp_send_not_observed'),
                ({'command_discarded':True},'safe_command_discarded'),
                ({'input':[0,0,0]},'received_zero'),
                ({'input_age_sec':.5},'safe_command_stale')]:
            with self.subTest(code=code):self.assertEqual(body_reason(dict(base,**changes)),code)

    def test_rate_limit_reports_short_states_without_log_flood(self):
        logger=Mock(); trace=MotionTrace(logger,'NAV_TEST')
        for i in range(201):
            trace.emit(dict(reason_code='terrain_unsafe' if i%2 else 'forwarded',
                source='nav',output=[0,0,0]),now=i*.05,active=True)
        self.assertLessEqual(logger.info.call_count,21)
        records=[json.loads(c.args[0].split(' ',1)[1]) for c in logger.info.call_args_list]
        self.assertTrue(any('terrain_unsafe' in r['window_reason_counts'] for r in records))
        self.assertGreaterEqual(logger.info.call_count,5)
        self.assertTrue(all(sum(r['window_reason_counts'].values())==r['window_sample_count'] for r in records))
        self.assertTrue(all(r['window_duration_sec']>=0 for r in records))

    def test_logger_failure_is_best_effort_and_json_is_finite(self):
        logger=Mock();trace=MotionTrace(logger,'NAV_TEST')
        self.assertTrue(trace.emit(dict(reason_code='forwarded',value=float('nan')),now=1,active=True))
        data=json.loads(logger.info.call_args.args[0].split(' ',1)[1])
        self.assertIsNone(data['value'])
        logger.info.side_effect=RuntimeError('diagnostic sink unavailable')
        self.assertFalse(trace.emit(dict(reason_code='other'),now=4,active=True))

    def test_xingji_actual_timer_records_unarmed_then_udp_dispatch_without_new_commands(self):
        from test_robot_body_selection import XingjiWorkerCallbacksTests
        f=XingjiWorkerCallbacksTests(methodName='runTest');f.setUp()
        n=f.node;n.navigation_diagnostic_active(NS(data=True))
        n.nav_velocity(f.command());n._control_tick()
        self.assertEqual(n.sent,[],'diagnostics cannot arm a body')
        self.assertEqual(n._motion_trace.last_record['reason_code'],'motion_not_armed')
        f.now+=.6;n.last_status_time=f.now
        f.arm();n.nav_velocity(f.command());n._control_tick()
        self.assertEqual(len(n.sent),1)
        self.assertEqual(n._motion_trace.last_record['reason_code'],'udp_dispatched_unconfirmed')
        self.assertTrue(n._motion_trace.last_record['send_success'])
        n._motion_trace.emit=Mock(side_effect=RuntimeError('log sink failed'))
        n.nav_velocity(f.command());n._control_tick()
        self.assertEqual(len(n.sent),2,'telemetry errors cannot suppress or add SDK commands')

    def test_normal_keepalive_does_not_report_a_false_no_pong_failure(self):
        source=Path(__file__).resolve().parents[1]/'car_vel/dog_remote_bridge.py'
        method=next(n for n in ast.walk(ast.parse(source.read_text(encoding='utf8')))
            if isinstance(n,ast.FunctionDef) and n.name=='_send_connection_ping')
        namespace={'time':time,'CONNECTION_PING':b'PING'}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method],type_ignores=[])),str(source),'exec'),namespace)
        logger=Mock()
        n=NS(_send_datagram=lambda _:True,state_lock=threading.RLock(),connection_ready=True,
            last_pong_time=time.monotonic(),connection_pings_sent=4,last_connection_ping_time=0,
            local_udp_port=45406,get_logger=lambda:logger)
        namespace['_send_connection_ping'](n)
        logger.warn.assert_not_called()
        n.connection_ready=False;n.connection_pings_sent=4
        namespace['_send_connection_ping'](n)
        self.assertEqual(logger.warn.call_count,1)

    def test_actual_body_tick_reports_feedback_loss_without_adding_emergency(self):
        from test_robot_body_selection import XingjiWorkerCallbacksTests
        f=XingjiWorkerCallbacksTests(methodName='runTest');f.setUp()
        n=f.node;f.arm();n.nav_velocity(f.command());n._control_tick()
        f.now+=.6;n.last_status_time=f.now
        n._navigation_gait_confirmed=Mock(return_value=False)
        n.nav_velocity(f.command());n._control_tick()
        self.assertEqual(n._motion_trace.last_record['reason_code'],'gait_feedback_invalid')
        self.assertEqual(n.sent[-1],(0,0,0))
        self.assertNotIn(('command',6),n.sent)
        self.assertFalse(n.nav_armed)

    def test_actual_body_records_send_failure_but_preserves_original_send_result(self):
        from test_robot_body_selection import XingjiWorkerCallbacksTests
        f=XingjiWorkerCallbacksTests(methodName='runTest');f.setUp()
        n=f.node;f.arm();n.nav_velocity(f.command())
        base=type(n).__mro__[1]
        with patch.object(base,'_send_velocity',return_value=False) as sender:
            n._control_tick()
            sender.assert_called_once()
        self.assertEqual(n._motion_trace.last_record['reason_code'],'udp_send_failed')
        self.assertFalse(n._motion_trace.last_record['execution_acknowledged'])
        self.assertTrue(n.nav_armed,'diagnostics must not change the existing authorization')


if __name__=='__main__':unittest.main()
