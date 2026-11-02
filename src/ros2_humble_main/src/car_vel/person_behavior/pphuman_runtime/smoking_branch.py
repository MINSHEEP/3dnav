"""Upstream v4 smoking association, using the pose worker's same-frame observations.

Fair sampling, real evidence times and identity resets are preserved. This
module neither opens a camera nor invokes the independent scene-fight model.
"""
import math
import numpy as np
from .runtime import NS, TimeWindow, current, empty_action
from .smoking_temporal import SmokingTemporal
from .smoking_hand_to_mouth import SmokingHandToMouth


class SmokingBranch:
    def __init__(self, cfg, backend):
        self.cfg, self.backend = cfg, backend
        temporal_cfg = self.cfg["smoking"].get("temporal", {})
        self.smoking_temporal = (SmokingTemporal(dict(temporal_cfg,
            result_ttl_seconds=self.cfg["smoking"]["result_ttl_seconds"]))
            if temporal_cfg.get("enabled", False) else None)
        self.smoking_clock = TimeWindow(self.cfg["smoking"])
        hand_cfg = self.cfg["smoking"].get("hand_to_mouth", {})
        self.smoking_hand = (SmokingHandToMouth(hand_cfg, self.cfg["smoking"].get("association"),
                             self.cfg["tracking"]["max_gap_seconds"]) if hand_cfg.get("enabled") else None)
        refinement = self.cfg['smoking'].get('profile_refinement', {})
        self.profile_smoking = (SmokingTemporal(dict(track_gap_seconds=self.cfg['tracking']['max_gap_seconds'],
                                confirmation_window_seconds=1.25, max_prediction_gap_seconds=1.25))
                                if refinement.get('enabled') else None)
        self.profile_strong = {}
        self.people = {}
        self.last_smoking_forward = None
        self.smoking_forward_count = 0

    def update(self, bgr, observations, stamp, epoch, call):
        for track_id in getattr(getattr(self.backend, "tracker", None), "retired_ids", ()):
            self.people.pop(track_id, None)
            if self.smoking_temporal is not None:
                self.smoking_temporal.forget(track_id)
            if self.smoking_hand is not None:
                self.smoking_hand.forget(track_id)
        seen = {p["track_id"] for p in observations}
        if self.profile_smoking is not None:
            self.profile_smoking.begin_frame(stamp, seen, epoch)
            for person in observations:
                if person.get('association', 'associated') != 'associated':
                    self.profile_smoking.forget(person['track_id'])
                    self.profile_strong.pop(person['track_id'], None)
            self.profile_strong = {i: t for i, t in self.profile_strong.items()
                                   if i in self.profile_smoking.tracks and stamp-t <= 1.25*NS}
            for track_id, state in self.people.items():
                cached = state.get('smoking') or {}
                if cached.get('reason') == 'profile_head_refinement' and track_id not in self.profile_smoking.tracks:
                    state['smoking'] = dict(empty_action('unknown', 'profile_sequence_invalidated', stamp))
        if self.smoking_hand is not None:
            invalidated = self.smoking_hand.begin_frame(stamp, observations, epoch)
            for track_id, state in self.people.items():
                cached = state.get("smoking") or {}
                if (track_id not in seen or (track_id in invalidated
                        and cached.get("reason") in ("cigarette_hand_to_mouth", "cigarette_near_face"))):
                    state["smoking"] = dict(empty_action("unknown", "hand_sequence_invalidated", stamp))
        if self.smoking_temporal is not None:
            self.smoking_temporal.begin_frame(stamp, seen, epoch)
            for track_id, state in self.people.items():
                if track_id not in seen:
                    state["smoking"] = dict(empty_action("unknown", "track_not_observed", stamp))
        for track_id, state in list(self.people.items()):
            if stamp - state["last_seen"] > self.cfg["tracking"]["max_gap_seconds"] * NS:
                del self.people[track_id]
                # Fall candidates expire independently and may survive a short,
                # explicitly verified tracker reassociation. Smoking does not.
        for p in observations:
            state = self.people.setdefault(p["track_id"], {"first_seen": stamp, "last_seen": stamp,
                                                         "last_attempt": None, "smoking": None})
            if ((self.smoking_temporal is not None or self.smoking_hand is not None)
                    and p.get("association", "associated") != "associated"):
                if self.smoking_temporal is not None:
                    self.smoking_temporal.forget(p["track_id"])
                if self.smoking_hand is not None:
                    self.smoking_hand.forget(p["track_id"])
                state["smoking"] = dict(empty_action("unknown", "track_association_changed", stamp))
            state["last_seen"] = stamp
        rgb = None

        def as_rgb():
            nonlocal rgb
            if rgb is None: rgb = np.ascontiguousarray(bgr[:, :, ::-1])
            return rgb

        smoke_cfg = self.cfg["smoking"]
        period = self.smoking_clock.period

        def eligible_since(track_id):
            state = self.people[track_id]
            return state["first_seen"] if state["last_attempt"] is None else state["last_attempt"] + period

        selected, wait_ms, interval = [], [], None
        if self.smoking_clock.due(stamp):
            # One clock governs every person, so newly appearing IDs cannot
            # accidentally cause additional GPU batches between global ticks.
            self.smoking_clock.add(stamp, None)
            ordered = sorted(observations, key=lambda p: (eligible_since(p["track_id"]), p["track_id"]))
            selected = ordered[:smoke_cfg["max_rois_per_tick"]]
            if selected:
                wait_ms = [max(0, stamp - eligible_since(p["track_id"])) / 1e6 for p in selected]
                crops = call("smoking_crops", self.backend.crops, as_rgb(),
                             [dict(p, raw_id=p["track_id"]) for p in selected])
                observations_by_id = {p["track_id"]: dict(p, keypoints_stamp_ns=stamp) for p in selected}
                results = call("smoking", self.backend.smoking, crops, observations_by_id, bgr.shape) if crops else {}
                if crops:
                    interval = (stamp - self.last_smoking_forward) / NS if self.last_smoking_forward is not None else None
                    self.last_smoking_forward = stamp
                    self.smoking_forward_count += 1
                for p, delay in zip(selected, wait_ms):
                    track_id = p["track_id"]
                    state = self.people[track_id]
                    state["last_attempt"] = stamp
                    if track_id not in results:
                        if self.profile_smoking is not None:
                            self.profile_smoking.forget(track_id)
                            self.profile_strong.pop(track_id, None)
                        if self.smoking_temporal is not None:
                            self.smoking_temporal.forget(track_id)
                        if self.smoking_hand is not None:
                            self.smoking_hand.forget(track_id)
                        state["smoking"] = dict(empty_action("unknown", "invalid_person_crop", stamp))
                        continue
                    raw = results[track_id]
                    if self.smoking_hand is not None:
                        raw = self.smoking_hand.update(track_id, raw, stamp,
                                                      association=p.get("association", "associated"))
                    refined = raw.get('evidence', {}).get('profile_refinement', {}).get('result')
                    if self.profile_smoking is not None:
                        if refined is None:
                            self.profile_smoking.forget(track_id); self.profile_strong.pop(track_id, None)
                        else:
                            association = p.get('association', 'associated')
                            if association != 'associated' or not refined.get('evidence', {}).get('head_observable'):
                                self.profile_strong.pop(track_id, None)
                            strong_threshold = smoke_cfg.get('profile_refinement', {}).get('threshold', smoke_cfg['threshold'])
                            if association in ('associated', 'new_track') and refined['label'] == 'smoking' and refined['score'] >= strong_threshold:
                                self.profile_strong[track_id] = stamp
                            if track_id not in self.profile_strong:
                                refined = dict(refined, label=None, score=None, state='unknown', reason='no_strong_profile_seed')
                            # One missed model tick does not erase genuine prior
                            # hits, but it emits NO positive or refreshed timestamp.
                            confirmed = (self.profile_smoking.update(track_id, refined, stamp, association=association)
                                         if refined['label'] == 'smoking' or track_id not in self.profile_strong
                                         else {'label': None})
                            raw['evidence']['profile_confirmation'] = dict(confirmed.get('temporal', {}),
                                strong_detection_stamp_ns=self.profile_strong.get(track_id))
                            if raw['label'] != 'smoking' and confirmed['label'] == 'smoking':
                                raw = dict(raw, label='smoking', state='valid', score=confirmed['score'],
                                           score_kind='temporally_associated_detection_score', reason='profile_head_refinement')
                    score = raw.get("score")
                    raw_state = raw.get("state", "valid")
                    if ((raw_state == "valid" and raw["label"] not in ("smoking", "other"))
                            or (raw_state == "unknown" and (raw["label"] is not None or score is not None))
                            or raw_state not in ("valid", "unknown") or (score is not None and
                            (not math.isfinite(score) or not 0 <= score <= 1))):
                        raise RuntimeError("Smoking predictor returned invalid result")
                    if self.smoking_temporal is not None:
                        raw = self.smoking_temporal.update(track_id, raw, stamp,
                            association=p.get("association", "associated"))
                        state["smoking"] = dict(raw, scheduling_wait_ms=delay)
                    else:
                        state["smoking"] = dict(raw, state=raw_state, updated=raw_state == "valid", prediction_stamp_ns=stamp,
                                                scheduling_wait_ms=delay)

        actions = {p['track_id']: current(self.people[p['track_id']]['smoking'], stamp, smoke_cfg)
                   for p in observations}
        schedule = dict(target_global_hz=smoke_cfg['sample_hz'],
                        max_rois_per_tick=smoke_cfg['max_rois_per_tick'],
                        selected_ids=[p['track_id'] for p in selected],
                        visible_people=len(observations), forward_count=self.smoking_forward_count,
                        last_forward_interval_seconds=interval, max_selected_wait_ms=max(wait_ms, default=0.),
                        max_visible_wait_ms=max([max(0, stamp - eligible_since(i)) / 1e6 for i in seen], default=0.))
        return actions, schedule
