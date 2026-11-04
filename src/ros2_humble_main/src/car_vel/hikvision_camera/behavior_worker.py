"""Run the supplied behavior worker with independently selectable branches."""
import os
import sys

sys.path.insert(0, os.getcwd())

from pphuman_runtime.runtime import TimeWindow, empty_action
from pphuman_runtime.runtime_yolo import RuntimeYolo


class BranchWindow(TimeWindow):
    def __init__(self, cfg, runtime, name):
        super().__init__(cfg)
        self.runtime, self.name = runtime, name

    def due(self, stamp):
        return self.runtime.enabled_branches[self.name] and super().due(stamp)


class FallRules:
    def __init__(self, rules, runtime):
        self.rules, self.runtime = rules, runtime

    def update(self, *args):
        if not self.runtime.enabled_branches['fall']:
            return empty_action('disabled')
        return self.rules.update(*args)

    def __getattr__(self, name):
        return getattr(self.rules, name)


class RobotRuntime(RuntimeYolo):
    def __init__(self, cfg, backend):
        self.enabled_branches = dict.fromkeys(('fall', 'fight', 'smoking'), True)
        super().__init__(cfg, backend)

    def reset(self, reason='explicit_reset'):
        super().reset(reason)
        self.smoking_clock = BranchWindow(self.cfg['smoking'], self, 'smoking')
        self.fight_window = BranchWindow(self.cfg['fight'], self, 'fight')
        self.rules = FallRules(self.rules, self)

    def infer(self, frame, header):
        self.enabled_branches = header.get('enabled_branches', self.enabled_branches)
        result = super().infer(frame, header)
        if not self.enabled_branches['fight']:
            result['scene']['fight'] = empty_action('disabled')
        if not self.enabled_branches['smoking']:
            for person in result['persons']:
                person['actions']['smoking'] = empty_action('disabled')
        actions = ([result['scene']['fight']] if self.enabled_branches['fight'] else [])
        actions += [person['actions'][key] for person in result['persons']
                    for key in ('fall', 'smoking') if self.enabled_branches[key]]
        result['status'] = ('partial' if any(a['state'] == 'unknown' for a in actions)
                            else 'warming' if any(a['state'] != 'valid' for a in actions) else 'valid')
        return result


if __name__ == '__main__':
    from pphuman_runtime import runtime_yolo, worker
    runtime_yolo.RuntimeYolo = RobotRuntime
    raise SystemExit(worker.main())
