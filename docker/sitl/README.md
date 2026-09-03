# ROS1/PX4 SITL environment

This optional environment pins Ubuntu 20.04, ROS Noetic, Gazebo 11, PX4
v1.13.3, and OSQP 0.6.3.

The repository is bind-mounted at `/catkin_ws/src/ommpc_controller`. Catkin
build and devel spaces use named volumes, so rebuilding the image does not add
generated files to the source tree. Docker Engine with the Compose v2 plugin is
the only host-side requirement.

From the repository root, build the image and catkin workspace:

```bash
docker compose -f docker/sitl/compose.yaml build
docker compose -f docker/sitl/compose.yaml run --rm ommpc-sitl ommpc-build
```

Start the service container and the headless PX4 Iris simulation:

```bash
docker compose -f docker/sitl/compose.yaml up -d ommpc-sitl
docker compose -f docker/sitl/compose.yaml exec -d ommpc-sitl ommpc-launch-px4
```

The container is headless and does not rely on the host display, host network,
or host ROS installation. To open a shell with ROS, PX4, Gazebo, and the built
workspace sourced, run:

```bash
docker compose -f docker/sitl/compose.yaml exec ommpc-sitl bash
```

The repository is available inside the container at
`/catkin_ws/src/ommpc_controller`.

Stop the environment after the trials with:

```bash
docker compose -f docker/sitl/compose.yaml down
```
