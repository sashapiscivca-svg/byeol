"""Prepare upload folders for the Hugging Face model repository (card + camera profiles) and dataset repository.

Usage (from the repository root):
    python scripts/stage_hf.py --github https://github.com/<org>/byeol [--out dist/hf]
Then upload, e.g.:
    huggingface-cli upload <org>/byeol dist/hf/model . --repo-type model
    huggingface-cli upload <org>/byeol-synth-v1 dist/hf/dataset . --repo-type dataset
Relative links in the cards are rewritten to absolute GitHub links, because the code does not live on the Hub.
There are no weights to upload: byeol is a deterministic algorithm.
"""
from __future__ import annotations

import argparse
import posixpath
import re
import shutil
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
LINK = re.compile(r"(\]\()([^)\s]+)(\))")


def rewrite(text: str, src_dir: str, github: str, keep: set[str]) -> str:
    """Rewrite relative Markdown links of a file located at `src_dir` (repo-relative) to GitHub URLs.

    Links whose resolved repo path is in `keep` stay relative (the file is uploaded next to the card)."""
    def fix(m: re.Match) -> str:
        target = m.group(2)
        if re.match(r"^[a-z]+:", target) or target.startswith("#"):
            return m.group(0)
        path, _, anchor = target.partition("#")
        resolved = posixpath.normpath(posixpath.join(src_dir, path)) if path else src_dir
        if resolved in keep:
            new = keep_name(resolved)
        else:
            kind = "tree" if (ROOT / resolved).is_dir() else "blob"
            new = f"{github.rstrip('/')}/{kind}/main/{resolved}"
        return f"{m.group(1)}{new}{'#' + anchor if anchor else ''}{m.group(3)}"
    return LINK.sub(fix, text)


def keep_name(repo_path: str) -> str:
    return {"MODEL_CARD.md": "README.md"}.get(repo_path, posixpath.basename(repo_path))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--github", required=True, help="public GitHub URL of this repository")
    ap.add_argument("--out", type=Path, default=ROOT / "dist" / "hf")
    a = ap.parse_args()
    model, data = a.out / "model", a.out / "dataset"
    for d in (model, data):
        if d.exists():
            shutil.rmtree(d)
        d.mkdir(parents=True)

    keep = {"MODEL_CARD.md", "MODEL_CARD.uk.md", "LICENSE", "ETHICS.md", "ETHICS.uk.md"}
    for name in ("MODEL_CARD.md", "MODEL_CARD.uk.md", "ETHICS.md", "ETHICS.uk.md"):
        text = rewrite((ROOT / name).read_text(encoding="utf-8"), "", a.github, keep)
        (model / keep_name(name)).write_text(text, encoding="utf-8")
    shutil.copy(ROOT / "LICENSE", model / "LICENSE")
    shutil.copytree(ROOT / "configs" / "cameras", model / "configs" / "cameras")

    src = ROOT / "data" / "byeol-synth-v1"
    for f in sorted(src.iterdir()):
        if f.name == "README.md":
            text = rewrite(f.read_text(encoding="utf-8"), "data/byeol-synth-v1", a.github, set())
            (data / "README.md").write_text(text, encoding="utf-8")
        else:
            shutil.copy(f, data / f.name)
    shutil.copy(ROOT / "LICENSE", data / "LICENSE")
    print(f"model repo folder:   {model}  ({sum(1 for _ in model.rglob('*') if _.is_file())} files)")
    print(f"dataset repo folder: {data}  ({sum(1 for _ in data.rglob('*') if _.is_file())} files)")


if __name__ == "__main__":
    main()
