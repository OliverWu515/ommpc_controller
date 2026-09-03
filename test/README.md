# PX4 SITL test runner

`sitl_test_runner.py` runs as a normal ROS1 node and does not depend on Docker.
It switches PX4 to OFFBOARD mode, performs takeoff, records timestamp-aligned
reference and vehicle states, and writes a JSON summary plus CSV samples. For a
polynomial trial it publishes the generated `PolyTraj`; for a
`horizontal-circle` trial it starts the text trajectory already loaded by the
controller configuration.

Each trial loads one complete controller configuration. The default polynomial
task uses [`polynomial_fixed_yaw.yaml`](../ommpc_ros/config/polynomial_fixed_yaw.yaml).
Use [`polynomial_variable_yaw.yaml`](../ommpc_ros/config/polynomial_variable_yaw.yaml) to
make the reference heading follow the polynomial path tangent.

Before recording a polynomial trial, the runner executes a smooth six-second
vertical prefix from the configured takeoff height to 0.7 m above it and back.
The prefix is intended to move the first in-air magnetic-heading reset observed
with the validated PX4 v1.13.3 setup ahead of the measured trajectory; it is not
a general guarantee against later estimator resets with other PX4 or EKF
configurations. The prefix is excluded from all reported metrics.

After the measured interval starts, the runner also excludes the first
`--warmup` seconds from its metrics; the default is `0.5`. The plotting script
uses the same default. Pass `--warmup 0` when invoking either Python script
directly if the entire measured interval must be included.

## Running without Docker

Unless an absolute path is shown, run the commands below from the repository
root.

Before starting a trial:

1. Install ROS1 and the controller dependencies.
2. Build the catkin workspace.
3. Source ROS and the workspace.
4. Start a ROS master, PX4 SITL, and MAVROS, and wait for MAVROS to connect.

The portable wrapper loads the task YAML, starts the controller, invokes the
runner, and stops the controller afterward:

```bash
source /opt/ros/noetic/setup.bash
source /path/to/catkin_ws/devel/setup.bash

/path/to/ommpc_controller/test/run_sitl_trial.sh \
  poly \
  /path/to/ommpc_controller/test/results/fixed_yaw_poly_v16.json \
  /path/to/ommpc_controller/ommpc_ros/config/polynomial_fixed_yaw.yaml \
  16.0
```

The positional arguments are:

1. Trajectory: `poly`, `horizontal-circle`, or the `text` alias.
2. Output JSON path.
3. Optional complete task YAML.
4. Optional polynomial peak speed in m/s; the default is `3.0`.
5. Optional shortened polynomial-trial duration in seconds. It has no effect
   on `horizontal-circle`, whose text duration is currently fixed at 27 s by
   the wrapper.

For `poly`, the default task YAML is
`ommpc_ros/config/polynomial_fixed_yaw.yaml`.
For `horizontal-circle`, it is `ommpc_ros/config/params.yaml`.

To invoke the Python runner directly, load the complete configuration and
start the controller before running it:

```bash
rosparam delete /ommpc_controller 2>/dev/null || true
rosparam load \
  /path/to/ommpc_controller/ommpc_ros/config/polynomial_fixed_yaw.yaml \
  /ommpc_controller

rosrun ommpc_ros ommpc_example_node __name:=ommpc_controller
```

In another sourced terminal:

```bash
rosrun ommpc_ros sitl_test_runner.py \
  --trajectory poly \
  --poly-peak-speed 16.0 \
  --output /path/to/result.json
```

The runner reads `vel_in_body`, `ref_txt/enable`, drag parameters, takeoff
height, and reference limits from the ROS parameter server. A missing required
parameter or a mismatch between the selected trajectory and `ref_txt/enable`
causes the trial to stop with an error.

At the end of a successful trial, the runner returns the controller to hover,
and the wrapper then stops the controller process. It does not command landing
or disarming. Start each repeatable trial from a fresh PX4 SITL instance; after
an interrupted trial, restart SITL before running another one.

## Running with Docker

Build the environment and workspace, then start headless PX4 SITL:

```bash
docker compose -f docker/sitl/compose.yaml build
docker compose -f docker/sitl/compose.yaml run --rm ommpc-sitl ommpc-build
docker compose -f docker/sitl/compose.yaml up -d ommpc-sitl
docker compose -f docker/sitl/compose.yaml exec -d ommpc-sitl ommpc-launch-px4
```

Run a trial from another terminal:

```bash
docker compose -f docker/sitl/compose.yaml exec ommpc-sitl \
  ommpc-run-trial poly \
  /catkin_ws/src/ommpc_controller/test/results/fixed_yaw_poly_v16.json \
  /catkin_ws/src/ommpc_controller/ommpc_ros/config/polynomial_fixed_yaw.yaml \
  16.0
```

Stop the environment afterward:

```bash
docker compose -f docker/sitl/compose.yaml down
```

See the [Docker README](../docker/sitl/README.md) for environment details.

## Outputs and plotting

The `run_sitl_trial.sh` wrapper creates:

- the requested JSON summary;
- a CSV file with the same stem;
- a controller log with the same stem and `.controller.log` suffix.

Invoking `sitl_test_runner.py` directly creates only the JSON and CSV files;
the separately started controller continues to write to its terminal unless
its output is explicitly redirected.

Generate a detailed plot with:

```bash
python3 test/plot_sitl_result.py \
  test/results/fixed_yaw_poly_v16.csv \
  --output misc/fixed_yaw_poly_v16.png \
  --title "16 m/s polynomial, fixed yaw"
```

The plotting script requires NumPy and Matplotlib and can run with or without
Docker.
