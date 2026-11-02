import sys
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from pphuman_runtime.smoking_temporal import SmokingTemporal


def action(label='smoking', score=.7):
    return {'label': label, 'score': score if label == 'smoking' else None,
            'state': 'unknown' if label is None else 'valid', 'updated': label is not None,
            'reason': 'face_unobservable' if label is None else 'spatial_result',
            'score_kind': 'detection_score' if label == 'smoking' else 'unavailable'}


class TemporalTests(unittest.TestCase):
    def setUp(self): self.gate = SmokingTemporal()
    def put(self, t, label='smoking', track=1, score=.7, **kw):
        return self.gate.update(track, action(label, score), round(t*1e9), **kw)

    def test_two_distinct_positives_confirm_and_keep_current_score(self):
        self.assertEqual(self.put(1)['state'], 'warming')
        r=self.put(1.5,score=.597)
        self.assertEqual(r['label'],'smoking'); self.assertEqual(r['score'],.597)
    def test_cached_repeats_never_confirm(self):
        self.put(1)
        for _ in range(20): self.assertEqual(self.put(1)['state'],'warming')
        self.assertEqual(self.gate.current(1,1_400_000_000)['temporal']['positive_observations'],1)
    def test_unknown_clears_confirmed_positive_even_updated_false(self):
        self.put(1);self.put(1.5);r=self.put(2,None)
        self.assertEqual(r['state'],'unknown');self.assertIsNone(r['label'])
        self.assertEqual(self.put(2.5)['state'],'warming')
    def test_negative_breaks_consecutive_confirmation(self):
        self.put(1);self.put(1.5,'other')
        self.assertEqual(self.put(2)['state'],'warming')
    def test_id_change_does_not_inherit(self):
        self.put(1);self.assertEqual(self.put(1.5,track=2)['state'],'warming')
    def test_new_track_reuse_clears_old_evidence(self):
        self.put(1);self.assertEqual(self.put(1.5,association='new_track')['state'],'warming')
    def test_prediction_gap_and_minimum_span(self):
        self.put(1);self.assertEqual(self.put(1.8)['state'],'warming')
        self.assertEqual(self.put(1.9)['state'],'warming')
        self.assertEqual(self.put(2.1)['label'],'smoking')
    def test_ttl_expiry_does_not_resurrect_on_duplicate(self):
        self.put(1);self.put(1.5)
        self.assertEqual(self.gate.current(1,3_000_000_001)['reason'],'expired')
        self.assertEqual(self.put(1.5)['reason'],'expired')
        self.assertEqual(self.put(3.1)['state'],'warming')
    def test_missing_person_forgets_immediately(self):
        self.gate.begin_frame(1_000_000_000,[1],1);self.put(1)
        self.gate.begin_frame(1_100_000_000,[],1)
        self.gate.begin_frame(1_200_000_000,[1],1)
        self.assertEqual(self.put(1.2)['state'],'warming')
    def test_epoch_and_frame_clock_reset(self):
        self.gate.begin_frame(1_000_000_000,[1],1);self.put(1)
        self.gate.begin_frame(1_200_000_000,[1],2)
        self.assertEqual(self.put(1.2)['state'],'warming')
        self.gate.begin_frame(1_100_000_000,[1],2)
        self.assertEqual(self.put(1.1)['state'],'warming')
    def test_out_of_order_prediction_clears_history(self):
        self.put(1);self.put(1.5)
        self.assertEqual(self.put(1.2)['reason'],'out_of_order_prediction')
        self.assertEqual(self.put(2)['state'],'warming')
    def test_unstable_association_is_unknown(self):
        r=self.put(1,association='ambiguous')
        self.assertEqual(r['state'],'unknown')
        self.assertEqual(r['raw_model_score'],.7);self.assertEqual(r['raw_spatial_label'],'smoking')
    def test_long_frame_gap_clears_even_if_id_still_visible(self):
        self.gate.begin_frame(1_000_000_000,[1],1);self.put(1)
        self.gate.begin_frame(1_500_000_000,[1],1)
        self.assertEqual(self.put(1.5)['state'],'warming')
    def test_current_cannot_add_votes_or_claim_updated(self):
        self.put(1);self.put(1.5)
        r=self.gate.current(1,1_500_000_000)
        self.assertEqual(r['label'],'smoking');self.assertFalse(r['updated'])
        self.assertEqual(r['temporal']['positive_observations'],2)
    def test_confirmed_positive_does_not_mask_next_unknown_cache(self):
        self.put(1);self.put(1.5);self.put(2,None)
        r=self.gate.current(1,2_400_000_000)
        self.assertEqual(r['state'],'unknown');self.assertIsNone(r['score'])
    def test_wrong_display_timestamp_is_rejected(self):
        raw=dict(action(),prediction_stamp_ns=1_000_000_000)
        with self.assertRaises(ValueError):self.gate.update(1,raw,1_500_000_000)
    def test_reset_forget_and_input_immutability(self):
        raw=action();self.gate.update(1,raw,1_000_000_000)
        self.assertNotIn('temporal',raw)
        self.gate.forget(1);self.assertEqual(self.put(1.5)['state'],'warming')
        self.gate.reset();self.assertEqual(self.put(2)['state'],'warming')
    def test_invalid_configuration(self):
        for cfg in [{'minimum_positive_observations':1},{'max_prediction_gap_seconds':2},{'result_ttl_seconds':0}]:
            with self.assertRaises(ValueError):SmokingTemporal(cfg)


if __name__ == '__main__': unittest.main()
