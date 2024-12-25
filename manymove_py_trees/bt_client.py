#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import py_trees
import py_trees_ros

from manymove_py_trees.move_definitions import (
    define_movement_configs,
    create_move,
    define_single_move_goal,
    define_sequence_move_goal
)
from manymove_py_trees.single_move_behavior import SingleMoveBehaviour
from manymove_py_trees.move_sequence_behavior import MoveSequenceBehaviour

import time
from geometry_msgs.msg import Pose, Point, Quaternion

def main():
    """
    Main entrypoint for our PyTrees-based client:
      1) Initialize rclpy
      2) Define moves/configs
      3) Build a root Sequence with single + sequence moves
      4) Tick the tree until completion
    """
    rclpy.init()

    node = rclpy.create_node("bt_client_node")
    node.get_logger().info("BT Client Node started")

    # --------------------------------------------------------------------------
    # 1) Define Movement Configurations
    # --------------------------------------------------------------------------
    movement_configs = define_movement_configs()

    # --------------------------------------------------------------------------
    # 2) Define Moves
    # --------------------------------------------------------------------------

    # Define target pose for single move
    single_target_pose = Pose()
    single_target_pose.position.x = 0.2
    single_target_pose.position.y = 0.0
    single_target_pose.position.z = 0.2
    single_target_pose.orientation.x = 1.0
    single_target_pose.orientation.y = 0.0
    single_target_pose.orientation.z = 0.0
    single_target_pose.orientation.w = 0.0

    # Create a single move
    single_move = create_move(
        movement_type="pose",
        target=single_target_pose,
        config=movement_configs["max_move"]
    )

    single_move_goal = define_single_move_goal(single_move)

    # Define a sequence of moves
    # Define various move targets
    rest_joint_values = [0.0, -0.785, 0.785, 0.0, 1.57, 0.0]
    scan_sx_joint_values = [-0.175, -0.419, 1.378, 0.349, 1.535, -0.977]
    scan_dx_joint_values = [0.733, -0.297, 1.378, -0.576, 1.692, 1.291]

    # Define sequence moves
    move1 = create_move(
        movement_type="joint",
        joint_values=rest_joint_values,
        config=movement_configs["mid_move"]
    )

    move2 = create_move(
        movement_type="joint",
        joint_values=scan_sx_joint_values,
        config=movement_configs["max_move"]
    )

    move3 = create_move(
        movement_type="joint",
        joint_values=scan_dx_joint_values,
        config=movement_configs["max_move"]
    )

    move4 = create_move(
        movement_type="named",
        named_target="home",
        config=movement_configs["mid_move"]
    )

    # Example pose move
    move5 = create_move(
        movement_type="pose",
        target=Pose(
            position=Point(x=0.2, y=-0.1, z=0.3),
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        ),
        config=movement_configs["mid_move"]
    )

    # Example cartesian move
    move6 = create_move(
        movement_type="cartesian",
        target=Pose(
            position=Point(x=0.2, y=-0.1, z=0.2),  # 0.3 - 0.1
            orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
        ),
        config=movement_configs["slow_move"]
    )

    # Reuse move1 for returning to rest
    move7 = move1

    # Assemble the sequence
    moves_sequence_01 = [move1, move2, move3, move4, move5, move6, move7]

    sequence_move_goal = define_sequence_move_goal(moves_sequence_01)

    # --------------------------------------------------------------------------
    # 3) Store move goals on the Blackboard
    # --------------------------------------------------------------------------
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.single_move_goal = single_move_goal
    blackboard.sequence_move_goal = sequence_move_goal

    # --------------------------------------------------------------------------
    # 4) Create Behaviors
    # --------------------------------------------------------------------------
    # Single Move Behavior
    single_move_behaviour = SingleMoveBehaviour(
        name="SingleMove",
        single_move_goal=blackboard.single_move_goal,
        node=node
    )

    # Sequence Move Behavior
    sequence_move_behaviour = MoveSequenceBehaviour(
        name="SequenceMove",
        sequence_goal=blackboard.sequence_move_goal,
        node=node,
        max_retries=3  # Optional: Set maximum retries for failed moves
    )

    # --------------------------------------------------------------------------
    # 5) Create a root Sequence with memory=True to remember state across ticks
    # --------------------------------------------------------------------------
    root = py_trees.composites.Sequence(name="RootSequence", memory=True)

    # Add child behaviors to the root sequence
    root.add_child(single_move_behaviour)
    root.add_child(sequence_move_behaviour)

    # --------------------------------------------------------------------------
    # 6) Build the BehaviourTree
    # --------------------------------------------------------------------------
    bt_tree = py_trees_ros.trees.BehaviourTree(root)

    # --------------------------------------------------------------------------
    # 7) Setup the tree
    # --------------------------------------------------------------------------
    try:
        bt_tree.setup(node=node, timeout=10.0)
    except Exception as e:
        node.get_logger().error(f"Failed to setup BehaviourTree: {e}")
        rclpy.shutdown()
        return

    # --------------------------------------------------------------------------
    # 8) Tick the tree until completion
    # --------------------------------------------------------------------------
    try:
        while rclpy.ok():
            # Tick the tree
            bt_tree.tick()

            # Check if the tree has reached a terminal state
            root_status = root.status
            if root_status == py_trees.common.Status.SUCCESS:
                node.get_logger().info("Behaviour Tree completed successfully.")
                break
            elif root_status == py_trees.common.Status.FAILURE:
                node.get_logger().error("Behaviour Tree failed.")
                break

            # Allow ROS 2 to process incoming messages
            rclpy.spin_once(node, timeout_sec=0.1)

            # Sleep for a short duration to control the tick rate
            time.sleep(0.1)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, exiting...")

    # --------------------------------------------------------------------------
    # 9) Clean up
    # --------------------------------------------------------------------------
    bt_tree.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
