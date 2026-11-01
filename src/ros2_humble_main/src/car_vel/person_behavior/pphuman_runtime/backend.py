"""Minimal PaddleDetection scene-fight adapter; no other predictors are constructed."""
import contextlib
import hashlib
import sys

@contextlib.contextmanager
def predictor_options(cfg):
    """Apply deployment IR options without editing pinned upstream sources.

    Upstream loaders do not expose delete_pass through their constructors. Scope
    the create_predictor hook to construction only; preserve official preprocess,
    predictor creation, forward and postprocess implementations.
    """
    previous = []
    for name in ("python.infer", "infer", "det_infer", "pphuman.video_action_infer"):
        module = sys.modules.get(name)
        if module is None or not hasattr(module, "create_predictor"):
            continue
        original = module.create_predictor

        def create(config, factory=original):
            for pass_name in cfg.get("disabled_ir_passes", []):
                config.delete_pass(pass_name)
            return factory(config)

        previous.append((module, original))
        module.create_predictor = create
    try:
        yield
    finally:
        for module, original in reversed(previous):
            module.create_predictor = original


def model_identity(path):
    files = sorted(p for p in path.iterdir()
                   if p.suffix in {".pdmodel", ".pdiparams", ".yml", ".yaml"})
    if not any(p.suffix == ".pdmodel" for p in files) or not any(
            p.suffix == ".pdiparams" for p in files):
        raise FileNotFoundError("Incomplete exported model: " + str(path))
    hashes = {}
    for file in files:
        digest = hashlib.sha256()
        with file.open("rb") as stream:
            for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                digest.update(chunk)
        hashes[file.name] = digest.hexdigest()
    return {"name": path.name, "path": str(path), "files_sha256": hashes}


class FightAdapter:
    def prepare_fight(self, rgb):
        # Official frame-list path expects already scaled RGB/PIL inputs.
        return self.scale(rgb)

    def fight(self, samples):
        classes, scores = self.fight_model.predict(samples)
        return int(classes[0]), float(scores[0])
