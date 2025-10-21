# manymove_py_trees

## DRAFT/EXAMPLE Repo

This repository is currently still a draft and serves as an example on how to leverage py_trees on ManyMove instead of 

## Overview
- Provides a `py_trees`-based framework for orchestrating manipulation sequences in the ManYMove stack.
- Wraps the `manymove_msgs/MoveManipulator` action in a reusable behaviour so planners can be built as trees instead of bespoke action clients.
- Includes helpers for composing `Move` objects and chaining them into behaviour-tree sequences.
- Works alongside the ManYMove planner action servers launched by other packages; see the top-level README for build and installation instructions.

## Modules
- `move_manipulator_behavior.MoveManipulatorBehavior` – leaf node that sends `MoveManipulator` goals, monitors feedback, handles cancellation/reset signals, and stores the resulting `RobotTrajectory` on the blackboard.
- `tree_helper.create_tree_from_moves()` / `tree_helper.create_tree_from_sequences()` – utilities that turn ordered `Move` lists into `py_trees_ros` behaviour trees with one `MoveManipulatorBehavior` per step.
- `move_definitions.Move` – dataclass describing a single move (pose, joint, named, or cartesian) plus helpers such as `create_move`, `create_pose`, and `define_movement_configs`.
- Example clients `bt_client_fake.py` and `bt_client_panda.py` show how to wire the helpers into full ROS 2 nodes that tick a tree while sharing an HMI service.

## Usage
- Install via apt `ros-$ROS_DISTRO-py-trees` and `ros-$ROS_DISTRO-py-trees-ros` packages
- For workspace setup, dependencies, and launch instructions, follow [ManyMove README](https://github.com/pastoriomarco/manymove/blob/main/README.md).
- Clone manymove_py_trees in the same src folder as ManyMove, build and source the workspace.
- You can find examples in manymove_py_trees/launch folder.

Available launchers:

```bash
ros2 launch manymove_bringup panda_movegroup_fake_py_trees.launch.py
```

```bash
ros2 launch manymove_bringup lite_movegroup_fake_py_trees.launch.py
```

## Credits

- **py_trees_ros** from [splintered-reality/py_trees_ros](https://github.com/splintered-reality/py_trees_ros), installed through ROS dependencies, including its visualizer **ros-humble-py-trees-ros-viewer**.

## Dependencies
- `rclpy`, `py_trees`, `py_trees_ros`
- `manymove_planner`, `manymove_msgs`
