import json
import tempfile
import unittest
from pathlib import Path

from lzp_service.environment_alerts import (
    EnvironmentAlertConfigStore,
    EnvironmentAlertPolicy,
    normalize_co2_threshold,
)


class EnvironmentAlertTests(unittest.TestCase):
    def test_config_is_validated_and_persisted_atomically(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'environment.json'
            store = EnvironmentAlertConfigStore(path)
            saved = store.set(1250)
            self.assertEqual(1250.0, saved['co2_threshold_ppm'])
            self.assertEqual(1, saved['version'])
            self.assertEqual(
                1250.0,
                EnvironmentAlertConfigStore(path).snapshot()[
                    'co2_threshold_ppm'])
            self.assertEqual(1250.0, json.loads(
                path.read_text(encoding='utf-8'))['co2_threshold_ppm'])
        for bad in (399, 10001, float('nan'), True, 'invalid'):
            with self.assertRaises(ValueError):
                normalize_co2_threshold(bad)

    def test_co2_requires_confirmation_then_uses_cooldown(self):
        policy = EnvironmentAlertPolicy(
            1000, confirmations=3, hysteresis_ppm=100,
            cooldown_seconds=60)
        self.assertEqual((), policy.observe_co2(
            1100, valid=True, active=True, now=0))
        self.assertEqual((), policy.observe_co2(
            1100, valid=True, active=True, now=1))
        self.assertEqual(('co2_high',), policy.observe_co2(
            1100, valid=True, active=True, now=2))
        self.assertEqual((), policy.observe_co2(
            1200, valid=True, active=True, now=40))
        self.assertEqual(('co2_high',), policy.observe_co2(
            1200, valid=True, active=True, now=62))

    def test_co2_recovery_has_hysteresis_and_can_retrigger(self):
        policy = EnvironmentAlertPolicy(
            1000, confirmations=2, hysteresis_ppm=100,
            cooldown_seconds=60)
        policy.observe_co2(1100, valid=True, active=True, now=0)
        self.assertEqual(('co2_high',), policy.observe_co2(
            1100, valid=True, active=True, now=1))
        policy.observe_co2(950, valid=True, active=True, now=2)
        self.assertTrue(policy.co2_alarm_active)
        policy.observe_co2(890, valid=True, active=True, now=3)
        policy.observe_co2(890, valid=True, active=True, now=4)
        self.assertFalse(policy.co2_alarm_active)
        policy.observe_co2(1100, valid=True, active=True, now=5)
        self.assertEqual(('co2_high',), policy.observe_co2(
            1100, valid=True, active=True, now=6))

    def test_invalid_or_inactive_co2_never_announces(self):
        policy = EnvironmentAlertPolicy(1000, confirmations=2)
        policy.observe_co2(1200, valid=True, active=True, now=0)
        self.assertEqual((), policy.observe_co2(
            1200, valid=False, active=True, now=1))
        self.assertEqual((), policy.observe_co2(
            1200, valid=True, active=False, now=2))
        self.assertEqual((), policy.observe_co2(
            1200, valid=True, active=True, now=3))

    def test_thermal_uses_detection_threshold_and_cooldown(self):
        policy = EnvironmentAlertPolicy(1000, cooldown_seconds=60)
        detection = {
            'measurement_type': 'temperature',
            'max_temperature_c': 61,
            'threshold_c': 60,
        }
        self.assertTrue(policy.observe_thermal(
            detection, active=True, now=0))
        self.assertFalse(policy.observe_thermal(
            detection, active=True, now=30))
        self.assertTrue(policy.observe_thermal(
            detection, active=True, now=60))
        self.assertFalse(policy.observe_thermal(
            detection, active=False, now=61))
        self.assertTrue(policy.observe_thermal(
            detection, active=True, now=62))


if __name__ == '__main__':
    unittest.main()
