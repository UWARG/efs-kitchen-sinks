# pyekf

Python implementation of Error-State Multiplicative Extended Kalman Filters (ES-MEKF) for UAV sensor fusion.
The math for each filter is written up in [`docs/`](docs/), and the filters are tested against simulated sensor
data in [`tests/`](tests/).

- **AHRS** (`ahrs_esmekf`): 9-state attitude filter using gyro, accelerometer, and magnetometer.
- **INS** (`ins_esmekf`): 18-state attitude, velocity, and position filter.

## Project Structure

```
ekf-python/
├── src/pyekf/
│   ├── ahrs_esmekf/
│   │   ├── AHRS_ESMEKF.py      # AHRS error-state filter (propagation, accel/mag corrections)
│   │   └── NominalState.py     # Quaternion propagation and error injection
│   ├── ins_esmekf/
│   │   ├── INS_ESMEKF.py       # INS error-state filter
│   │   └── NominalState.py     # Quaternion, velocity, displacement propagation and error injection
│   ├── Measurements.py         # Current/previous sensor readings and accumulated bias estimates
│   ├── quaternions.py          # Quaternion algebra and rotation matrices
│   └── utils.py
├── tests/                      # pytest suite + simulator, see tests/README.md
├── docs/
│   ├── ahrs_esmekf.md          # Math behind the AHRS filter, from Kalman filter basics
│   └── ins_esmekf.md           # Math behind the INS filter
├── plots/                      # Test output plots (gitignored)
├── CHANGELOG_INS.md
└── pyproject.toml
```

## Setup

Requires Python 3.12+ and [`uv`](https://docs.astral.sh/uv/) (`pip install uv`).

```bash
uv venv
uv pip install -e ".[dev]"
```

## Testing

```bash
uv run pytest                               # run all tests (plots saved to plots/)
uv run pytest tests/test_ahrs_esmekf.py -s  # one file, with printed output
```

Mission Planner tests need a live SITL connection and are skipped by default. See [`tests/README.md`](tests/README.md).
