"""CPU association tests and --replay mode for the entire frozen development cache."""
import hashlib
import json
from pathlib import Path
import sys
import unittest
from collections import Counter

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pphuman_runtime.smoking_filter import classify_smoking, DEFAULTS


def face():
    points = np.zeros((17, 3))
    points[:7] = [[100, 60, .95], [110, 50, .95], [90, 50, .95],
                  [125, 55, .9], [75, 55, .9], [140, 110, .95], [60, 110, .95]]
    return points


def detection(score=.8, bbox=(96, 70, 106, 76)):
    return {"class": 0, "score": score, "bbox": list(bbox)}


class SmokingFilterTests(unittest.TestCase):
    def classify(self, ds, points=None, **cfg):
        return classify_smoking(ds, face() if points is None else points, [40, 20, 160, 220], cfg)

    def test_mouth_positive_preserves_exact_model_score(self):
        result = self.classify([detection(.5970708727836609)])
        self.assertEqual(result['label'], 'smoking')
        self.assertEqual(result['score'], .5970708727836609)

    def test_threshold_boundary_and_no_fake_negative_probability(self):
        self.assertEqual(self.classify([detection(.5)])['label'], 'smoking')
        result = self.classify([detection(.499999)])
        self.assertEqual(result['label'], 'other'); self.assertIsNone(result['score'])
        self.assertEqual(result['evidence']['raw_max_score'], .499999)

    def test_wall_in_expanded_crop_is_not_this_person(self):
        result = self.classify([detection(.99, (180, 60, 190, 70))])
        self.assertEqual(result['label'], 'other')
        self.assertEqual(result['evidence']['highest_rejected']['reason'], 'outside_person')

    def test_arm_candidate_is_rejected_inside_person(self):
        result = self.classify([detection(.8, (135, 140, 145, 150))])
        self.assertEqual(result['evidence']['highest_rejected']['reason'], 'outside_mouth_neighborhood')

    def test_top_background_does_not_hide_lower_mouth_candidate(self):
        ds = [detection(.99, (180, 60, 190, 70)), detection(.55)]
        result = self.classify(ds, max_evidence_candidates=1)
        self.assertEqual(result['score'], .55)
        self.assertEqual(result['evidence']['selected_detection']['index'], 1)
        self.assertEqual(result['evidence']['highest_rejected']['score'], .99)
        self.assertTrue(result['evidence']['diagnostics_truncated'])

    def test_no_face_is_unknown_even_with_no_candidates(self):
        result = self.classify([], np.zeros((17, 3)))
        self.assertEqual(result['state'], 'unknown'); self.assertIsNone(result['label'])

    def test_no_detector_observation_or_truncated_candidates_is_unknown(self):
        self.assertEqual(self.classify(None)['reason'], 'detector_not_observed')
        self.assertEqual(self.classify([detection()], candidates_complete=False)['state'], 'unknown')

    def test_complete_empty_detection_is_other_when_face_observable(self):
        self.assertEqual(self.classify([])['label'], 'other')

    def test_off_person_extrapolated_face_is_unknown(self):
        p = face(); p[0, 0] = 200
        self.assertEqual(self.classify([], p)['state'], 'unknown')

    def test_profile_requires_shoulders_for_orientation(self):
        p = face(); p[2, 2] = 0; p[4, 2] = 0
        self.assertEqual(self.classify([], p)['state'], 'valid')
        p[5:7, 2] = 0
        self.assertEqual(self.classify([], p)['state'], 'unknown')

    def test_rotation_and_scale_preserve_association(self):
        p = face(); box = np.array([96, 70, 106, 76.])
        # Rotate 90 degrees and scale twofold; head is no longer world-upright.
        p[:, :2] = np.column_stack([300 - 2*p[:, 1], 2*p[:, 0]])
        rotated = [300-2*box[3], 2*box[0], 300-2*box[1], 2*box[2]]
        result = classify_smoking([detection(.8, rotated)], p, [-140,80,260,320], {})
        self.assertEqual(result['label'], 'smoking')

    def test_invalid_model_score_and_wrong_class(self):
        with self.assertRaises(ValueError): self.classify([detection(float('nan'))])
        d = detection(); d['class'] = 1
        self.assertEqual(self.classify([d])['label'], 'other')

    def test_inputs_are_not_mutated(self):
        p = face(); before = p.copy(); ds = [detection()]; saved = json.dumps(ds)
        self.classify(ds, p)
        np.testing.assert_array_equal(p, before); self.assertEqual(json.dumps(ds), saved)

    def test_all_candidates_examined_before_diagnostic_top_k(self):
        ds = [detection(.99, (180, 60, 190, 70)) for _ in range(99)] + [detection(.55)]
        result = self.classify(ds, max_evidence_candidates=1)
        self.assertEqual(result['label'], 'smoking')
        self.assertEqual(result['evidence']['candidate_count'], 100)
        self.assertEqual(result['evidence']['selected_detection']['index'], 99)

