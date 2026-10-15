# Contributing to byeol

**English** · [Українська](CONTRIBUTING.uk.md)

Thank you for helping. byeol exists to help find people in danger, and contributions of every size matter. By contributing you agree that your contribution is licensed under the [MIT License](LICENSE) and that you have read [ETHICS.md](ETHICS.md).

## What helps most

1. **Real flight recordings.** This is what the project lacks most. Synchronised IMU, flight-controller state, height, gimbal angle and video, recorded under a permissive licence (CC BY 4.0 or CC0), ideally over water. Follow [docs/RECORDING_PROTOCOL.md](docs/RECORDING_PROTOCOL.md) and open an issue to discuss how to share the data.
2. **Measurements on target hardware.** Run `python scripts/bench_budget.py` on a Raspberry Pi Zero 2 W or a similar board and report the output, the board, the OS and the numpy version.
3. **Calibrated camera profiles.** Profiles from real calibrations (`scripts/calibrate_camera.py`), with the camera model, resolution, date and calibration RMS. Do not copy manufacturer specifications without a primary source.
4. **Log readers** for PX4 ULog, ArduPilot DataFlash or MAVLink tlog that convert into the stream format used by `scripts/eval_dataset.py`.
5. **Bug reports** with a minimal reproduction (inputs, `EgoConfig`, expected and actual `EgoResult`).

## Development workflow

```bash
pip install -e ".[dev]"
python -m pytest tests -q          # all tests must pass
ruff check .                       # lint must be clean
python scripts/make_dataset.py --check   # if you touched synth.py, see below
```

- Keep the core (`src/byeol`) dependent on **numpy only**. Optional dependencies must be imported lazily and fail with a clear message.
- Every new module and script must be listed in [docs/MODULES.md](docs/MODULES.md); a test checks this.
- Inside the estimator, use SI units, NED/FRD frames and a single clock. Put conversions at the boundary (`units.py`, `frames.py`, `timebase.py`).
- Add tests for new behaviour. For geometry, prefer analytic checks against closed-form cases.
- **Do not change published numbers silently.** If a change affects results, re-run the relevant script in [docs/TECHNICAL_REPORT.md](docs/TECHNICAL_REPORT.md) §8, update `experiments/` and the docs in both languages, and explain the change in the pull request and in [CHANGELOG.md](CHANGELOG.md).
- If you change `synth.py`, the dataset is no longer reproducible as v1. Bump the dataset version instead of overwriting `byeol-synth-v1`.
- Significant design changes get a new entry in [docs/DESIGN_DECISIONS.md](docs/DESIGN_DECISIONS.md) with the reason and the alternatives.

## Documentation

All user-facing documentation is bilingual: English (`*.md`) and Ukrainian (`*.uk.md`). Update both in the same pull request, or say in the pull request that a translation is still needed. Write each paragraph and list item as a single line; do not hard-wrap prose.

## Pull requests

- One topic per pull request, with a clear description of what changed and why.
- Report honestly: include negative results and the conditions in which your change does not help.
- Be respectful. See [CODE_OF_CONDUCT.md](CODE_OF_CONDUCT.md).

## Security and misuse

Do not open public issues for vulnerabilities or misuse reports. See [SECURITY.md](SECURITY.md).
