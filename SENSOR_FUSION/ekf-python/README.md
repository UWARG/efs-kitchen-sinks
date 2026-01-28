# pyekf

A Python library for implementing and experimenting with Error-State Extended Kalman Filters (ES-EKF) for UAV Sensor Fusion.

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
