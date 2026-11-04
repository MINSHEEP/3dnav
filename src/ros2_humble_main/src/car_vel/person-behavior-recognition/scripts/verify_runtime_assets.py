"""Verify the assets shipped with the default three-model profile; no GPU needed."""
import argparse
import hashlib
import json
from pathlib import Path


def verify(root, manifest_path="assets/runtime_assets.json"):
    root = Path(root).resolve()
    manifest = json.loads((root / manifest_path).read_text(encoding="utf-8"))
    checked = []
    for entry in manifest["files"]:
        path = (root / entry["path"]).resolve()
        path.relative_to(root)
        if not path.is_file():
            raise ValueError(f"Missing asset: {entry['path']}. Restore the deployment archive, or run git lfs pull for a Git checkout.")
        with path.open("rb") as stream:
            if stream.read(128).startswith(b"version https://git-lfs.github.com/spec/v1"):
                raise ValueError(f"LFS pointer, not model bytes: {entry['path']}. Run git lfs pull.")
            stream.seek(0)
            digest = hashlib.sha256()
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
        if path.stat().st_size != entry["bytes"] or digest.hexdigest() != entry["sha256"]:
            raise ValueError(f"Asset size/SHA256 mismatch: {entry['path']}")
        checked.append(entry["path"])
    return {"status": "PASS", "files": len(checked), "bytes": sum(f["bytes"] for f in manifest["files"]),
            "checked": checked}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--manifest", default="assets/runtime_assets.json", help="Asset manifest, relative to root")
    args = parser.parse_args()
    try:
        print(json.dumps(verify(args.root, args.manifest), ensure_ascii=False, indent=2))
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, str(exc) + "\n")


if __name__ == "__main__":
    main()
