# Tests

## Structure

```
tests/
├── conftest.py                 # Shared fixtures (plots_dir, home_dir) and the --run-mission-planner option
├── test_quaternions.py         # Unit tests for quaternion algebra
├── test_nominal_state.py       # INS nominal state propagation vs. closed-form solution
├── test_ahrs_esmekf.py         # AHRS filter, simulated (reference for how filter tests are written)
├── test_ins_esmekf.py          # INS filter, simulated
├── test_mission_planner_ahrs.py# AHRS against live Mission Planner / SITL data (manual only)
│
├── sim/
│   ├── trajectory.py           # Trajectory abstract base class (ground truth API)
│   ├── constant_trajectory.py  # Constant linear acceleration + constant angular velocity, closed form
│   ├── sensors.py              # SensorSimulator: turns ground truth into noisy, biased gyro/accel/mag readings
│   └── params.py               # SensorParams and ConstantSimParams dataclasses
│
├── utils/
│   ├── utils.py                # Assertions: quaternion closeness, attitude error, covariance validity, n-sigma checks
│   └── grapher.py              # ResultsCollector (attitude plots), INSResultsCollector (+ velocity/position plots)
│
└── mission_planner/
    └── drone.py                # pymavlink wrapper used by the Mission Planner test
```

## How the simulated tests work

Every filter test is set up the same way:

1. **Params**: `SensorParams` (noise and bias variances) and `ConstantSimParams` (dt, duration, initial state,
   constant motion, gravity and magnetic field).
2. **Ground truth**: `ConstantMotionTrajectory.get_state(t)` returns displacement, velocity, acceleration,
   quaternion, and angular velocity in the inertial frame at any time `t`, in closed form.
3. **Sensors**: `SensorSimulator.get_readings(t)` rotates the truth into the body frame and returns
   gyro, accelerometer (specific force `R^T (a - g)`), and magnetometer (`R^T m`) readings. Biases are constant offsets
   drawn once from `N(0, bias_cov)`; noise is drawn from `N(0, cov)` each call. A fixed `seed` makes runs repeatable.
   `SensorSimulator.get_gps_readings(t)` returns noisy GPS position and velocity in the inertial frame.
4. **Filter**: initialized from the readings at `t = 0` and the true initial state.
5. **Loop**: for each time step, `state_extrapolation(...)`, then any `correction_<sensor>(...)`, then
   `grapher.collect(...)`.
6. **Plot**: `grapher.save_and_show(output_dir=plots_dir, ...)` writes a PNG to `plots/`.
7. **Assert**: check the final estimate is close enough to the truth.

Tests go from easiest to hardest: (1) no noise, no correction (integration must be exact),
(2) noise, no correction (drift expected), (3) noise + corrections, (4) noise + bias + corrections.
The INS also has (5) to (7), which add GPS and check attitude, velocity, and position accuracy, bias estimates,
and a fast spinning case.

Things to keep in mind when writing asserts:

- Only assert on things the sensors can actually measure. The magnetometer can't see rotation about the magnetic
  field direction, and without GPS the INS can't correct its position, velocity, or accel bias.
- `ConstantMotionTrajectory` only rotates about one axis, so some sensor biases along that axis look the same as an
  attitude error.
- `assert_covariance_valid` checks the covariance stays symmetric positive semi-definite, and
  `assert_error_within_sigma` checks the real error is within 3σ of what the filter thinks it is.
- The simulator uses `cov` as the variance of each sample, but the filters use it as a continuous noise density
  (`Q ≈ cov·dt`). The AHRS tests pass it straight through, so the AHRS thinks it is less accurate than it is. The INS
  tests convert it (`cov·dt` for gyro and accel noise, and `bias_cov` as the initial bias uncertainty with almost no
  bias random walk) so the INS covariance can be checked properly.

## Running

```bash
uv run pytest                                          # everything except Mission Planner
uv run pytest tests/test_ins_esmekf.py -s              # one file with prints
uv run pytest tests/test_ins_esmekf.py -k no_noise -s  # one test
```

## Mission Planner Test (manual)

1. Open Mission Planner, go to the Simulation tab and start SITL.
2. Press `Ctrl+F` → MAVLink, connect to host port 14450, connection UDP, host 127.0.0.1, needs write access, click Go.
3. Set `CONNECTION_STRING=udpin:127.0.0.1:14450` in `.env`.
4. Actions → Arm, then right click the map to take off and to fly to waypoints.
5. Run `uv run pytest tests/test_mission_planner_ahrs.py --run-mission-planner -s`.
