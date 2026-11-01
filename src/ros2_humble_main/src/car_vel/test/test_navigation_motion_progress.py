"""Read-only motion progress checks with fresh odometry, never robot I/O."""
import math
from pathlib import Path
import sys
import unittest
import json
from types import SimpleNamespace as NS
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from car_vel.navigation_motion_progress import NavigationMotionProgress


class MotionProgressTests(unittest.TestCase):
    def setUp(self):
        self.monitor = NavigationMotionProgress()

    def sample(self, t, *, stamp=None, x=0., y=0., yaw=0., frame='odom'):
        self.monitor.observe(now=t, stamp=t if stamp is None else stamp,
            frame=frame, child_frame='base_footprint', x=x, y=y, yaw=yaw)

    def tick(self, t, *, enabled=True, command=(.1, 0., 0.), context=('route', 1)):
        return self.monitor.evaluate(now=t, ros_now=t, enabled=enabled,
            command=command, context=context)

    def stationary(self, start=100., duration=6., **kwargs):
        for i in range(round(duration*10)+1):
            t=start+i*.1
            self.sample(t)
            result=self.tick(t, **kwargs)
        return result

    def test_stationary_fresh_samples_warn_after_five_seconds(self):
        self.assertFalse(self.stationary(duration=4.9)['warning'])
        self.sample(105.)
        result=self.tick(105.)
        self.assertEqual(result['state'], 'no_motion_observed')
        self.assertTrue(result['warning'])
        self.assertIn('里程计未检测到明显移动', result['reason'])
        self.assertTrue(result['read_only'])

    def test_real_translation_and_rotation_each_clear_warning(self):
        for motion in ('translation', 'rotation'):
            with self.subTest(motion=motion):
                self.monitor=NavigationMotionProgress()
                self.assertTrue(self.stationary()['warning'])
                self.sample(106.1, x=.05 if motion=='translation' else 0.,
                    yaw=.1 if motion=='rotation' else 0.)
                result=self.tick(106.1)
                self.assertFalse(result['warning'])
                self.assertEqual(result['state'], 'progress_observed')

    def test_turn_in_place_is_measured_by_yaw(self):
        for i in range(101):
            t=100+i*.1
            self.sample(t, yaw=i*.01)
            result=self.tick(t, command=(0.,0.,.1))
            self.assertFalse(result['warning'])

    def test_noise_is_not_accumulated_as_distance(self):
        for i in range(61):
            t=100+i*.1
            self.sample(t, x=.004*(1 if i%2 else -1), yaw=.005*(1 if i%2 else -1))
            result=self.tick(t)
        self.assertEqual(result['state'], 'no_motion_observed')

    def test_duplicate_timestamp_does_not_refresh_odometry(self):
        for i in range(61):
            t=100+i*.1
            self.sample(t, stamp=100.)
            result=self.tick(t)
        self.assertEqual(result['state'], 'odometry_unavailable')
        self.assertNotIn('未检测到明显移动', result['reason'])
        self.assertGreater(result['odom_age_sec'], 1.)

    def test_stale_source_missing_and_invalid_data_cannot_claim_stall(self):
        self.assertEqual(self.tick(100.)['state'], 'odometry_unavailable')
        self.sample(100., stamp=90.)
        self.assertEqual(self.tick(100.)['state'], 'odometry_unavailable')
        self.sample(101., x=math.nan)
        self.assertEqual(self.tick(101.)['state'], 'odometry_unavailable')

    def test_frame_time_or_pose_reset_restarts_evidence_window(self):
        for reset in ('frame', 'stamp', 'jump', 'clock'):
            with self.subTest(reset=reset):
                self.monitor=NavigationMotionProgress()
                self.stationary(duration=4.9)
                now=104.95 if reset!='clock' else 95.
                self.sample(now, stamp=99. if reset=='stamp' else now,
                    frame='new_odom' if reset=='frame' else 'odom',
                    x=3. if reset=='jump' else 0.)
                result=self.tick(now)
                self.assertFalse(result['state']=='no_motion_observed')
                self.assertLessEqual(result['duration_sec'], .001)

    def test_zero_disabled_or_context_change_discards_old_window(self):
        for kwargs in ({'command':(0.,0.,0.)}, {'enabled':False}, {'context':('route',2)}):
            with self.subTest(kwargs=kwargs):
                self.monitor=NavigationMotionProgress()
                self.stationary(duration=4.9)
                self.sample(105.)
                self.assertFalse(self.tick(105., **kwargs)['warning'])
                self.sample(105.1)
                self.assertFalse(self.tick(105.1)['warning'])

    def test_tick_gap_cannot_be_counted_as_continuous_output(self):
        self.sample(100.);self.tick(100.)
        self.sample(110.)
        self.assertFalse(self.tick(110.)['warning'])

    def test_warning_does_not_remain_after_odometry_disappears(self):
        self.assertTrue(self.stationary()['warning'])
        result=self.tick(107.1)
        self.assertEqual(result['state'], 'odometry_unavailable')
        self.sample(107.2)
        self.assertFalse(self.tick(107.2)['warning'])

    def test_yaw_wrap_is_not_a_reset_or_large_rotation(self):
        for i in range(61):
            t=100+i*.1
            self.sample(t, yaw=math.pi-.001 if i%2 else -math.pi+.001)
            result=self.tick(t)
        self.assertEqual(result['state'], 'no_motion_observed')


class MotionProgressCallbacksTests(unittest.TestCase):
    def setUp(self):
        from test_velocity_arbiter_callbacks import VelocityArbiterCallbacksTest
        self.fixture=VelocityArbiterCallbacksTest(methodName='runTest')
        self.fixture.setUp()
        self.fixture.body('xingji')
        self.node=self.fixture.node
        self.node._navigation_active_callback(NS(data=True))

    def odometry(self, *, x=0., yaw=0., stamp=None):
        t=self.fixture.now if stamp is None else stamp
        return NS(header=NS(stamp=NS(sec=int(t), nanosec=round((t-int(t))*1e9)), frame_id='odom'),
            child_frame_id='base_footprint', pose=NS(pose=NS(
                position=NS(x=x,y=0.), orientation=NS(x=0.,y=0.,z=math.sin(yaw/2),w=math.cos(yaw/2)))))

    def cycle(self, *, odom=True, x=0., yaw=0.):
        f=self.fixture;f.now+=.05
        self.node._lio_pause_callback(NS(data=False))
        self.node._navigation_pause_callback(NS(data=False))
        f.body(self.node._body_id)
        self.node._global_localization_callback(NS(data=json.dumps(dict(
            ready=True, scope='xingji_3d', body_generation=1, stamp_ns=int(f.now*1e9)))))
        f.terrain(True);f.command()
        if odom:self.node._progress_odometry_callback(self.odometry(x=x,yaw=yaw))
        self.node._tick()

    def guard(self):
        return json.loads(self.node._guard_publisher.publish.call_args.args[0].data)

    def test_final_output_preserved_and_warning_reaches_existing_guard_topic(self):
        for _ in range(121):self.cycle()
        data=self.guard()
        self.assertTrue(data['warning'])
        self.assertEqual(data['warning_code'],'no_motion_observed')
        self.assertFalse(data['blocked'])
        self.assertEqual(self.fixture.last_velocity(),(.2,.1))
        self.assertIn('NAV_PROGRESS',str(self.node.log.warning.call_args_list))
        self.cycle(x=.1)
        self.assertFalse(self.guard()['warning'])

    def test_2d_gangben_inactive_and_teleop_never_gain_stall_warning(self):
        for scope in ('2d','gangben','inactive','teleop'):
            with self.subTest(scope=scope):
                self.setUp()
                if scope=='2d':self.fixture.mode('map_2d')
                if scope=='gangben':self.fixture.body('gangben_zsl1')
                if scope=='inactive':self.node._navigation_active_callback(NS(data=False))
                for _ in range(121):
                    if scope=='teleop':self.node._teleop_callback(self.fixture.module.Twist())
                    self.cycle()
                self.assertFalse(self.guard()['warning'])
                self.assertEqual(self.guard()['motion_progress']['state'],'inactive')

    def test_diagnostic_and_guard_publisher_failures_cannot_suppress_velocity(self):
        for _ in range(110):self.cycle()
        self.node._progress_monitor.evaluate=Mock(side_effect=RuntimeError('diagnostic failure'))
        self.node._guard_publisher.publish=Mock(side_effect=RuntimeError('publisher failure'))
        self.node.log.warning=Mock(side_effect=RuntimeError('logger failure'))
        before=self.node.publisher.publish.call_count
        self.cycle()
        self.assertEqual(self.node.publisher.publish.call_count,before+1)
        self.assertEqual(self.fixture.last_velocity(),(.2,.1))

    def test_route_point_switch_and_active_cycle_reset_warning(self):
        for _ in range(121):self.cycle()
        self.assertTrue(self.guard()['warning'])
        self.node._route_diagnostic_callback(NS(route_id=3,route_name='1',map_name='room',
            point_index=1,point_count=19,current_loop=1,state='navigating',mode='route'))
        self.cycle()
        self.assertFalse(self.guard()['warning'])
        for _ in range(121):self.cycle()
        self.assertTrue(self.guard()['warning'])
        self.node._navigation_active_callback(NS(data=False))
        self.node._navigation_active_callback(NS(data=True))
        self.cycle()
        self.assertFalse(self.guard()['warning'])

    def test_invalid_or_missing_odometry_reports_unavailable_not_physical_stall(self):
        for _ in range(121):self.cycle(odom=False)
        data=self.guard()
        self.assertTrue(data['warning'])
        self.assertEqual(data['warning_code'],'odometry_unavailable')
        self.assertEqual(self.fixture.last_velocity(),(.2,.1))

    def test_short_stop_or_mode_transition_between_ticks_resets_evidence(self):
        for change in ('zero','pause','mode','body'):
            with self.subTest(change=change):
                self.setUp()
                for _ in range(121):self.cycle()
                self.assertTrue(self.guard()['warning'])
                if change=='zero':self.node._nav_callback(self.fixture.module.Twist())
                elif change=='pause':
                    self.node._navigation_pause_callback(NS(data=True))
                    self.node._navigation_pause_callback(NS(data=False))
                elif change=='mode':
                    self.fixture.mode('map_2d');self.fixture.mode('terrain_3d')
                else:
                    self.fixture.body('gangben_zsl1');self.fixture.body('xingji')
                self.cycle()
                self.assertFalse(self.guard()['warning'])

    def test_stable_warning_logs_once_and_does_not_change_motion_count(self):
        for _ in range(241):self.cycle()
        messages=[str(call.args[0]) for call in self.node.log.warning.call_args_list]
        self.assertEqual(sum('NAV_PROGRESS' in message for message in messages),1)
        self.assertEqual(self.node.publisher.publish.call_count,242)  # body change emits one zero


if __name__=='__main__': unittest.main()
