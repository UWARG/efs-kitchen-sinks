# pyekf

A Python library for implementing and experimenting with Error-State Extended Kalman Filters (ES-EKF) for UAV Sensor Fusion.

## Project Structure

```
pyekf/
├── src/pyekf/
│   ├── __init__.py
│   ├── ESMEKF.py                   # Error-State Multiplicative Extended Kalman Filter implementation
│   ├── NominalState.py             # Nominal state propagation and correction
│   ├── Measurements.py             # Sensor measurement handling and bias tracking
│   ├── quaternions.py              # Quaternion operations and utilities
│   └── utils.py                    # Utility functions (vectors, matrices, constants)
│
├── tests/
│   ├── sim/
│   └── mission_planner/
│
├── pyproject.toml
├── .python-version
├── .gitignore
└── README.md
```

### Key Components

- **ESMEKF**: Core extended Kalman filter for sensor fusion combining gyroscope, accelerometer, and magnetometer data
- **NominalState**: Manages position, velocity, and attitude (quaternion) propagation
- **Measurements**: Stores and updates sensor readings with bias accumulation
- **Quaternions**: Implements quaternion algebra operations essential for 3D rotation representation

## Installation

### Prerequisites

Install `uv` (a fast Python package manager):

```bash
pip install uv
```

### Create a virtual environment

```bash
uv venv
```

### Install pyekf

```bash
uv pip install .
```

### Development install (with tests)

```bash
uv pip install -e .[dev]
uv run pytest
```
