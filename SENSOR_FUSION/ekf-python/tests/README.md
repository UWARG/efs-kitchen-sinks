# Tests

## Structure

```
├── tests/
│   ├── sim/
│   │   ├── __init__.py
│   │   ├── sensors.py              # SensorSimulator class for generating IMU/Mag readings
│   │   └── trajectory.py           # ConstantMotionTrajectory class for ground truth generation
│   │
│   ├── mission_planner/
│   │   ├── __init__.py
│   │   └── drone.py                # Drone-specific mission configurations and parameters
│   │
│   ├── test_sim_scenarios.py       # Integration tests using simulation classes
│   ├── test_es_mekf.py             # ESMEKF filter unit tests
│   ├── test_nominal_state.py       # Nominal state propagation unit tests
│   └── test_quaternions.py         # Quaternion operation unit tests
```
