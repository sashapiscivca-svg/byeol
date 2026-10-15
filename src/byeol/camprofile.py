"""Camera profiles in YAML/JSON: loading, validation with clear error messages, saving.

Schema (schema: 1): name, kind (rgb|lwir), notes, source, license, resolution{width,height},
intrinsics{mode: pixels|mm|fov, ...}, distortion{k1,k2,p1,p2,k3}. Unknown keys are an error (so that typos are not silently ignored).
YAML requires pyyaml (optional); JSON works without dependencies.
"""
from __future__ import annotations

import json
from pathlib import Path

from .cameramodel import CameraConfigError, CameraModel, Distortion

TOP = {"schema", "name", "kind", "notes", "source", "license", "resolution", "intrinsics", "distortion"}
MODES = {
    "pixels": ({"mode", "fx", "fy", "cx", "cy"}, {"fx", "fy"}),
    "mm": ({"mode", "focal_mm", "sensor_w_mm", "sensor_h_mm", "cx", "cy"}, {"focal_mm", "sensor_w_mm", "sensor_h_mm"}),
    "fov": ({"mode", "hfov_deg", "vfov_deg", "cx", "cy"}, {"hfov_deg"}),
}


def _num(d: dict, key: str, where: str) -> float:
    v = d[key]
    if isinstance(v, bool) or not isinstance(v, (int, float)):
        raise CameraConfigError(f"{where}.{key} must be a number, got {v!r}")
    return float(v)


def load_camera_dict(d: dict) -> CameraModel:
    if not isinstance(d, dict):
        raise CameraConfigError("profile must be a mapping")
    if d.get("schema") != 1:
        raise CameraConfigError(f"schema: only 1 is supported, got {d.get('schema')!r}")
    extra = set(d) - TOP
    if extra:
        raise CameraConfigError(f"unknown profile keys: {sorted(extra)} (allowed: {sorted(TOP)})")
    res = d.get("resolution")
    if not isinstance(res, dict):
        raise CameraConfigError("missing section resolution: {width, height}")
    for k in ("width", "height"):
        if not isinstance(res.get(k), int) or isinstance(res.get(k), bool):
            raise CameraConfigError(f"resolution.{k} must be an integer, got {res.get(k)!r}")
    w, h = res["width"], res["height"]
    intr = d.get("intrinsics")
    if not isinstance(intr, dict):
        raise CameraConfigError("missing section intrinsics: {mode: pixels|mm|fov, ...}")
    mode = intr.get("mode")
    if mode not in MODES:
        raise CameraConfigError(f"intrinsics.mode must be one of {sorted(MODES)}, got {mode!r}")
    allowed, required = MODES[mode]
    bad = set(intr) - allowed
    if bad:
        raise CameraConfigError(f"intrinsics (mode={mode}): unexpected fields {sorted(bad)}; allowed {sorted(allowed)}")
    miss = required - set(intr)
    if miss:
        raise CameraConfigError(f"intrinsics (mode={mode}): missing fields {sorted(miss)}")
    dd = d.get("distortion", {}) or {}
    badd = set(dd) - {"k1", "k2", "p1", "p2", "k3"}
    if badd:
        raise CameraConfigError(f"distortion: unknown coefficients {sorted(badd)}")
    dist = Distortion(**{k: _num(dd, k, "distortion") for k in dd})
    kw = {"dist": dist, "kind": d.get("kind", "rgb"), "name": str(d.get("name", "")), "notes": str(d.get("notes", ""))}
    ctr = {k: _num(intr, k, "intrinsics") for k in ("cx", "cy") if k in intr}
    n = {k: _num(intr, k, "intrinsics") for k in intr if k != "mode" and k not in ("cx", "cy")}
    if mode == "pixels":
        return CameraModel.from_pixels(w, h, n["fx"], n["fy"], **ctr, **kw)
    if mode == "mm":
        return CameraModel.from_mm(w, h, n["focal_mm"], n["sensor_w_mm"], n["sensor_h_mm"], **ctr, **kw)
    return CameraModel.from_fov(w, h, n["hfov_deg"], n.get("vfov_deg"), **ctr, **kw)


def _read(path: Path) -> dict:
    text = path.read_text(encoding="utf-8")
    if path.suffix.lower() == ".json":
        return json.loads(text)
    try:
        import yaml
    except ImportError as e:
        raise CameraConfigError(".yaml requires pyyaml (pip install pyyaml), or use .json") from e
    return yaml.safe_load(text)


def load_camera(path: str | Path) -> CameraModel:
    p = Path(path)
    if not p.is_file():
        raise CameraConfigError(f"profile file not found: {p}")
    try:
        d = _read(p)
    except CameraConfigError:
        raise
    except Exception as e:  # YAML/JSON syntax error
        raise CameraConfigError(f"{p}: failed to parse file: {e}") from e
    try:
        m = load_camera_dict(d)
    except CameraConfigError as e:
        raise CameraConfigError(f"{p}: {e}") from e
    return m


def dump_camera(m: CameraModel, path: str | Path, extra: dict | None = None) -> None:
    """Saves the profile in pixels mode (the most precise); extension .json or .yaml."""
    d = {"schema": 1, "name": m.name, "kind": m.kind, "notes": m.notes, **(extra or {}),
         "resolution": {"width": m.width, "height": m.height},
         "intrinsics": {"mode": "pixels", "fx": m.fx, "fy": m.fy, "cx": m.cx, "cy": m.cy},
         "distortion": {"k1": m.dist.k1, "k2": m.dist.k2, "p1": m.dist.p1, "p2": m.dist.p2, "k3": m.dist.k3}}
    p = Path(path)
    p.parent.mkdir(parents=True, exist_ok=True)
    if p.suffix.lower() == ".json":
        p.write_text(json.dumps(d, ensure_ascii=False, indent=1), encoding="utf-8")
        return
    try:
        import yaml
    except ImportError as e:
        raise CameraConfigError("writing .yaml requires pyyaml; use .json") from e
    p.write_text(yaml.safe_dump(d, allow_unicode=True, sort_keys=False), encoding="utf-8")
