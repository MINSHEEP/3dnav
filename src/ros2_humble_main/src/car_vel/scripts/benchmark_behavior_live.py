#!/usr/bin/env python3
"""Read existing gateway metrics; never start models, change settings or move the robot."""
import argparse
from collections import Counter
import json
from pathlib import Path
import statistics
import time
import urllib.request


def run():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--url', default='http://127.0.0.1:8766')
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if not 1 <= args.seconds <= 3600:
        parser.error('seconds must be between 1 and 3600')
    samples = {key: [] for key in ('fall', 'fight', 'smoking')}
    states = {key: Counter() for key in samples}
    seen, stale_start, latest = {}, {}, {}
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        with urllib.request.urlopen(args.url.rstrip('/') + '/api/behavior/status', timeout=5) as response:
            latest = json.load(response)
        for key, branch in latest['branches'].items():
            states[key][branch['state']] += 1
            stale_start.setdefault(key, branch.get('stale_results', 0))
            identity = (latest.get('source_epoch'), latest.get('sequence'), latest.get('stale_results', 0))
            latency = latest.get('last_result_latency_seconds')
            if identity != seen.get(key) and latency is not None:
                samples[key].append(latency * 1000)
                seen[key] = identity
        time.sleep(.1)
    report = {'seconds': args.seconds, 'enabled': latest['enabled'], 'workers': {'project': {'running': latest['running'], 'ready': latest['ready']}},
              'semantics': 'Runtime latency/state sampling, not recognition accuracy. No extra inference processes.',
              'branches': {}}
    for key, values in samples.items():
        ordered = sorted(values)
        report['branches'][key] = {
            'count': len(values), 'p50_ms': statistics.median(values) if values else None,
            'p95_ms': ordered[min(len(ordered)-1, int(.95*len(ordered)))] if ordered else None,
            'states': dict(states[key]), 'last_status': latest['branches'][key],
            'new_expired_results': latest['branches'][key].get('stale_results', 0)-stale_start[key]}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    run()
