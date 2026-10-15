from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
DOC = ROOT / "docs" / "MODULES.md"


def test_every_module_is_documented():
    doc = DOC.read_text(encoding="utf-8")
    missing = []
    for p in sorted((ROOT / "src" / "byeol").rglob("*.py")):
        if p.name == "__init__.py":
            continue
        rel = p.relative_to(ROOT / "src" / "byeol").as_posix()
        if rel not in doc:
            missing.append(rel)
    assert not missing, f"Modules missing from docs/MODULES.md: {missing}"


def test_every_script_is_documented():
    doc = DOC.read_text(encoding="utf-8")
    missing = [p.name for p in sorted((ROOT / "scripts").glob("*.py")) if p.name not in doc]
    assert not missing, f"Scripts missing from docs/MODULES.md: {missing}"
