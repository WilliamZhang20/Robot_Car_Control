# Path Optimization Simulation

An extension of [this](https://github.com/WilliamZhang20/ros-task) repository, but now with simulation of more complex dynamic obstacle avoidance algorithms.

## Outline

1. Configured working LiDAR with a new `/scan` topic.
2. todo: Implementation of Timed Elastic Band (TEB) and/or Model Predictive Path Integral controllers for path optimization

## How to Run

### Build the simulator

```bash
./scripts/build/sim.sh
```

### Run the simulator

```bash
./scripts/deploy/devel.sh # To enter the docker container
ros2 launch limo_simulation limo.launch.py # To launch the simulator
```
