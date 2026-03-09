# Tests

## Structure

```
├── tests/
│   ├── conftest.py
│   ├── README.md
│   ├── test_ahrs_esmekf_old.py
│   ├── test_ahrs_esmekf.py
│   ├── test_ins_esmekf.py
│   ├── test_mission_planner.py
│   ├── test_nominal_state.py
│   ├── test_quaternions.py
│   ├── __pycache__/
│   │
│   ├── mission_planner/
│   │   ├── drone.py                # Drone-specific mission configurations and parameters
│   │   ├── README.md
│   │   └── __pycache__/
│   │
│   └── sim/
│       ├── constant_trajectory.py
│       ├── multi_axis_varying_rotation_trajectory.py
│       ├── params.py
│       ├── sensors.py              # SensorSimulator class for generating IMU/Mag readings
│       ├── trajectory.py           # Trajectory classes for ground truth generation
│       └── __pycache__/
```

## Mission Planner Test

1. Open missionplanner, go sim
2. ctrl f and mavlink, connect to host 14450, connection udp, host 127.0.0.1, needs write access, click go
3. set `CONNECTION_STRING=udpin:127.0.0.1:14450` in .env
4. actions, arm, then right click to take off and right click to go places.
5. add `--run-mission-planner` to pytest command to run mission planner tests