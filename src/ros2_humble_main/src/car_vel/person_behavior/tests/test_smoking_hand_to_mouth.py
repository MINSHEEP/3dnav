"""Causal sequence and real runtime wiring; synthetic inputs are NOT accuracy tests."""
import copy
from pathlib import Path
import unittest

import numpy as np

from pphuman_runtime.smoking_filter import classify_smoking
from pphuman_runtime.smoking_hand_to_mouth import SmokingHandToMouth
from pphuman_runtime.runtime_yolo import RuntimeYolo, load_config
from pphuman_runtime.yolo_backend import YoloBackend

ROOT = Path(__file__).resolve().parents[1]


def person(near=False, other_hand=False, **extra):
    points = np.zeros((17, 3))
    points[:7] = [[100, 60, .95], [110, 50, .95], [90, 50, .95],
                  [125, 55, .9], [75, 55, .9], [140, 110, .95], [60, 110, .95]]
    points[9] = [150, 200, .99]
    points[10] = [50, 200, .99]
    if near:
        points[10 if other_hand else 9] = [115, 110, .99]
    return dict(track_id=1, bbox=[20, 20, 180, 260], keypoints=points, score=.9,
                association='associated', **extra)


def detection(score=.7, box=(145, 180, 155, 190), cls=0):
    return dict(score=score, bbox=list(box), **{'class':cls})


class SequenceTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(ROOT/'config/inference.yaml')['smoking']
        self.engine = SmokingHandToMouth(self.cfg['hand_to_mouth'], self.cfg['association'])

    def frame(self, seconds, p=None, ds=None, epoch=0):
        p = p or person()
        stamp = round(seconds*1e9)
        invalidated = self.engine.begin_frame(stamp, [p], epoch)
        raw = classify_smoking(ds or [], p['keypoints'], p['bbox'],
            dict(self.cfg['association'], threshold=self.cfg['threshold'], hand_to_mouth=self.cfg['hand_to_mouth']))
        return self.engine.update(1, raw, stamp, p['association']), invalidated

    def approach(self, p=None):
        self.frame(.2)
        a,_ = self.frame(.4, p or person(True))
        self.assertNotEqual(a['label'], 'smoking')
        return self.frame(.6, p or person(True))[0]

    def test_actual_detection_then_same_hand_approach_has_honest_score_and_age(self):
        a,_ = self.frame(0, ds=[detection()])
        self.assertEqual(a['label'], 'other')  # Holding alone cannot alarm.
        a = self.approach()
        self.assertEqual((a['label'],a['score'],a['reason']), ('smoking',.7,'cigarette_hand_to_mouth'))
        self.assertEqual(a['score_kind'], 'temporally_associated_detection_score')
        seq = a['evidence']['hand_to_mouth']
        self.assertEqual((seq['detection_stamp_ns'],seq['pose_stamp_ns'],seq['evidence_age_seconds']), (0,600000000,.6))
        # A completed event ends immediately when the hand leaves the mouth.
        a,invalidated = self.frame(.8)
        self.assertEqual(invalidated, {1})
        self.assertNotEqual(a['label'], 'smoking')

    def test_no_smoke_weak_score_wrong_class_or_wrong_hand_do_not_confirm(self):
        for ds in ([], [detection(.49)], [detection(cls=1)]):
            with self.subTest(ds=ds):
                self.setUp(); self.frame(0, ds=ds)
                self.assertNotEqual(self.approach()['label'], 'smoking')
        self.setUp(); self.frame(0, ds=[detection()])
        self.assertNotEqual(self.approach(person(True, other_hand=True))['label'], 'smoking')

    def test_observation_loss_epoch_time_and_identity_changes_discard_seed(self):
        for kind in ('face', 'wrist', 'track', 'epoch', 'gap', 'backwards', 'ambiguous'):
            with self.subTest(kind=kind):
                self.setUp(); self.frame(0, ds=[detection()])
                p=person(True)
                if kind=='face': p['keypoints'][:5,2]=0
                if kind=='wrist': p['keypoints'][9,2]=0
                if kind=='ambiguous': p['association']='ambiguous'
                self.engine.begin_frame(400000000 if kind=='gap' else 0 if kind=='backwards' else 100000000,
                    [] if kind=='track' else [p], 1 if kind=='epoch' else 0)
                self.assertFalse(self.engine.tracks)
                self.assertNotEqual(self.approach()['label'], 'smoking')

    def test_no_unbounded_alarm_or_memory_from_cached_detector(self):
        a,_=self.frame(0, ds=[detection()])
        self.approach()
        for step in range(4,17):
            a,_=self.frame(step*.2, person(True))
        self.assertFalse(self.engine.tracks)
        self.assertNotEqual(a['label'],'smoking')
        with self.assertRaises(ValueError):
            self.engine.update(1, dict(a,prediction_stamp_ns=0),3200000000)

    def test_all_candidates_checked_and_ambiguous_wrist_is_not_seeded(self):
        ds=[detection(.9, (250,180,260,190)) for _ in range(21)]+[detection(.5)]
        p=person()
        a=classify_smoking(ds,p['keypoints'],p['bbox'],dict(hand_to_mouth=self.cfg['hand_to_mouth'],max_evidence_candidates=1))
        self.assertEqual(a['evidence']['candidate_decisions'][0]['score'],.9)
        self.assertEqual(a['evidence']['hand_candidates'][0]['score'],.5)
        p['keypoints'][10]=p['keypoints'][9]
        a=classify_smoking([detection()],p['keypoints'],p['bbox'],dict(hand_to_mouth=self.cfg['hand_to_mouth']))
        self.assertEqual(a['evidence']['hand_candidates'],[])

    def test_already_raised_hand_needs_strong_then_distinct_matching_detection(self):
        p=person(True)
        ds=[detection(.8,(142,70,152,76))]
        a,_=self.frame(0,p,ds)
        self.assertNotEqual(a['label'],'smoking')
        self.assertNotEqual(self.engine.update(1,a,0)['label'],'smoking')
        self.frame(.1,p)
        a,_=self.frame(.2,p,[detection(.4,(140,70,150,76))])
        self.assertEqual((a['label'],a['score'],a['reason']),('smoking',.4,'cigarette_near_face'))
        sequence=a['evidence']['hand_to_mouth']
        self.assertEqual(sequence['strong_detection']['score'],.8)
        self.assertEqual(sequence['detection_stamp_ns'],200000000)
        self.assertFalse(sequence['short_occlusion'])

    def test_near_face_weak_scores_alone_or_one_high_score_cannot_confirm(self):
        for scores in ((.4,.4,.4),(.8,0,0)):
            self.setUp()
            for i,score in enumerate(scores):
                a,_=self.frame(i*.2,person(True),[detection(score,(142,70,152,76))] if score else [])
                self.assertNotEqual(a['label'],'smoking')

    def test_near_face_other_hand_or_shifted_object_cannot_confirm(self):
        for p,box in ((person(True,other_hand=True),(142,70,152,76)),(person(True),(50,70,60,76))):
            self.setUp();self.frame(0,person(True),[detection(.8,(142,70,152,76))])
            a,_=self.frame(.2,p,[detection(.4,box)])
            self.assertNotEqual(a['label'],'smoking')

    def test_near_face_occlusion_is_bounded_and_does_not_renew_detector_time(self):
        self.frame(0,person(True),[detection(.8,(142,70,152,76))])
        self.frame(.2,person(True),[detection(.4,(140,70,150,76))])
        for i in range(2,7):
            a,_=self.frame(i*.2,person(True))
            self.assertEqual(a['label'],'smoking')
            self.assertEqual(a['evidence']['hand_to_mouth']['detection_stamp_ns'],200000000)
            self.assertTrue(a['evidence']['hand_to_mouth']['short_occlusion'])
        a,invalidated=self.frame(1.21,person(True))
        self.assertEqual(invalidated,{1})
        self.assertNotEqual(a['label'],'smoking')

    def test_near_face_repeated_weak_scores_cannot_extend_strong_evidence_lifetime(self):
        self.frame(0,person(True),[detection(.8,(142,70,152,76))])
        for i in range(1,16):
            a,_=self.frame(i*.2,person(True),[detection(.4,(140,70,150,76))])
            self.assertEqual(a['label'],'smoking')
        a,_=self.frame(3.01,person(True),[detection(.4,(140,70,150,76))])
        self.assertNotEqual(a['label'],'smoking')

    def test_near_face_loss_reset_and_opt_out(self):
        for kind in ('face','wrist','track','epoch','gap','ambiguous','away'):
            with self.subTest(kind=kind):
                self.setUp();self.frame(0,person(True),[detection(.8,(142,70,152,76))])
                self.frame(.2,person(True),[detection(.4,(140,70,150,76))])
                p=person(True)
                if kind=='face':p['keypoints'][:5,2]=0
                if kind=='wrist':p['keypoints'][9,2]=0
                if kind=='ambiguous':p['association']='ambiguous'
                if kind=='away':p=person()
                invalidated=self.engine.begin_frame(600000000 if kind=='gap' else 300000000,
                    [] if kind=='track' else [p],1 if kind=='epoch' else 0)
                self.assertEqual(invalidated,{1});self.assertFalse(self.engine.tracks)
        self.setUp();self.cfg['hand_to_mouth']['near_face']['enabled']=False
        self.engine=SmokingHandToMouth(self.cfg['hand_to_mouth'],self.cfg['association'])
        for i in range(3):
            a,_=self.frame(i*.2,person(True),[detection(.9,(142,70,152,76))])
            self.assertNotEqual(a['label'],'smoking')


class RecordedShapeBackend:
    """Exercises backend coordinate conversion and runtime clock, with no weights."""
    identities = {k: {'name': k} for k in ('yolo_pose','smoking','fight')}
    last_timing = {}

    def __init__(self,cfg):
        self.cfg=cfg; self.calls=0; self.smoking_model=self

    def reset(self): pass
    def observe(self, frame, stamp):
        self.stamp=stamp
        return [person(stamp>=600000000)]
    def crops(self, rgb, people):
        return {p['track_id']:(rgb,[0,0,300,300],p['bbox']) for p in people}
    def predict_image(self,images,visual=False):
        self.calls+=1
        rows=[[0,.7,145,180,155,190]] if self.stamp==0 else []
        return {'boxes_num':[len(rows)],'boxes':np.asarray(rows).reshape(-1,6)}
    smoking=YoloBackend.smoking
    def prepare_fight(self,rgb): return 0
    def fight(self,samples): return (0,.8)


class RuntimeWiringTests(unittest.TestCase):
    def test_near_face_runtime_clears_cache_between_detector_ticks(self):
        cfg=load_config(ROOT/'config/inference.yaml')
        class NearBackend(RecordedShapeBackend):
            def observe(self,frame,stamp):
                self.stamp=stamp
                return [person(True)]
            def predict_image(self,images,visual=False):
                self.calls+=1
                score=.8 if self.stamp==0 else .4 if self.stamp==500000000 else 0
                rows=[[0,score,142,70,152,76]] if score else []
                return dict(boxes_num=[len(rows)],boxes=np.asarray(rows).reshape(-1,6))
        backend=NearBackend(cfg);runtime=RuntimeYolo(cfg,backend)
        actions=[]
        for i in range(18):
            r=runtime.infer(np.zeros((300,300,3),np.uint8),dict(stamp_ns=i*100000000,source_epoch=0,request_id=i))
            actions.append(r['persons'][0]['actions']['smoking'])
        self.assertEqual(backend.calls,4)
        self.assertEqual(actions[5]['reason'],'cigarette_near_face')
        self.assertFalse(actions[6]['updated'])
        self.assertEqual(actions[15]['label'],'smoking')
        self.assertNotEqual(actions[16]['label'],'smoking')

    def test_forward_budget_ttl_reset_and_opt_out(self):
        cfg=load_config(ROOT/'config/inference.yaml')
        backend=RecordedShapeBackend(cfg); runtime=RuntimeYolo(cfg,backend)
        frame=np.zeros((300,300,3),np.uint8)
        def run(t,epoch=0):
            result=runtime.infer(frame,dict(stamp_ns=round(t*1e9),source_epoch=epoch,request_id=int(t*10)))
            return result['persons'][0]['actions']['smoking']
        actions=[run(i/10) for i in range(34)]
        self.assertTrue(any(a['label']=='smoking' for a in actions))
        self.assertEqual(backend.calls,7)  # Same 2 Hz forwards; no extra models.
        self.assertEqual(actions[10]['prediction_stamp_ns'],1000000000)
        self.assertTrue(actions[10]['updated'])
        self.assertFalse(actions[11]['updated'])
        self.assertNotEqual(actions[31]['label'],'smoking')  # Memory age, not cache TTL.
        self.assertNotEqual(run(3.4,1)['label'],'smoking')
        cfg=copy.deepcopy(cfg);cfg['smoking']['hand_to_mouth']['enabled']=False
        runtime=RuntimeYolo(cfg,RecordedShapeBackend(cfg))
        self.assertFalse(any(run(i/10)['label']=='smoking' for i in range(15)))


if __name__=='__main__': unittest.main()
