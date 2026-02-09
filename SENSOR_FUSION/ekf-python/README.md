# pyekf

A Python library for implementing and experimenting with Error-State Extended Kalman Filters (ES-EKF) for UAV Sensor Fusion.

Currently supports gyroscope, accelerometer, and magnetometer sensors only.

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
│   ├── mission_planner/
│   └── test_bunch_of_stuff.py
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

## Testing (TODO)

## TODOS

1. all todos in code
2. fast inverse of psd matrices (3x3), also if check for singularity
3. new testing framework + experiement class for logging + visualizing data
4. make compatible with mission planner tests?
5. outline assumptions, e.g. initial cov of sensors always independent across dims (diagonal), always col vectors, mag inertial, etc.

6. rearchitecture -> data struct for nominal state to use in tests, experiment class for printing, saving, ploting experiments, generic enough so can handle mission planner

7. if adding more measurements, error state can be non zero then we need to account for it in predicting measurements, e.g. small angle error in magnetometer
