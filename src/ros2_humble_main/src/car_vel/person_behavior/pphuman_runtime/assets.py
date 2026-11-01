"""Verify the supplied assets and safely unpack the pinned deploy source.

No setup script, archive program, symlink or executable in the source archive
is executed. Existing files with different contents are never overwritten.
"""
import argparse
import hashlib
import json
import shutil
import tarfile
from pathlib import Path, PurePosixPath


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def verify(root):
    manifest = json.loads((root / "assets/runtime_assets.json").read_text(encoding="utf-8"))
    for item in manifest["files"]:
        path = (root / item["path"]).resolve()
        if root.resolve() not in path.parents:
            raise ValueError("Asset escapes runtime root")
        if not path.is_file() or path.stat().st_size != item["bytes"] or sha256(path) != item["sha256"]:
            raise ValueError("Missing, truncated or changed asset: " + item["path"])
    return manifest


def prepare_vendor(root, destination):
    manifest = verify(root)
    archive = next(item for item in manifest["files"] if item.get("role") == "pinned_vendor_source_archive")
    destination = destination.resolve()
    destination.mkdir(parents=True, exist_ok=True)
    with tarfile.open(root / archive["path"], "r:gz") as source:
        members = source.getmembers()
        prefixes = {PurePosixPath(member.name).parts[0] for member in members}
        if len(prefixes) != 1:
            raise ValueError("Expected one archive root")
        for member in members:
            relative = PurePosixPath(member.name)
            if relative.is_absolute() or ".." in relative.parts or "\\" in member.name:
                raise ValueError("Unsafe archive path")
            relative = Path(*relative.parts[1:])
            if not relative.parts:
                continue
            target = (destination / relative).resolve()
            if destination not in target.parents:
                raise ValueError("Archive target escapes destination")
            # Upstream has README links; they are not needed for inference.
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            elif member.isfile():
                data = source.extractfile(member)
                target.parent.mkdir(parents=True, exist_ok=True)
                if target.exists():
                    expected = hashlib.sha256(data.read()).hexdigest()
                    if not target.is_file() or sha256(target) != expected:
                        raise ValueError("Existing vendor file differs; choose a clean destination: " + str(target))
                else:
                    with target.open("xb") as stream:
                        shutil.copyfileobj(data, stream)
        if not (destination / "deploy/pipeline/pphuman/video_action_infer.py").is_file():
            raise ValueError("Archive does not contain the required deploy source")
    return destination


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prepare-vendor", type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    manifest = verify(root)
    if args.prepare_vendor:
        prepare_vendor(root, args.prepare_vendor)
    print(json.dumps({"verified_files": len(manifest["files"]),
                      "active_models": manifest["active_models"],
                      "vendor": str(args.prepare_vendor) if args.prepare_vendor else None}))


if __name__ == "__main__":
    main()
