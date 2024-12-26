#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import py_trees
import py_trees_ros

from manymove_py_trees.move_definitions import (
    define_movement_configs,
    create_move,
    define_single_move_goal,
)
from manymove_py_trees.planning_behavior import PlanningBehaviour
from manymove_py_trees.execution_behavior import ExecuteTrajectoryBehaviour

from geometry_msgs.msg import Pose, Point, Quaternion

import time

def main():
    """
    Main entrypoint for our PyTrees-based client:
      1) Initialize rclpy
      2) Define moves/configs
      3) Build a root Sequence with planning and execution
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
    # Define two sets of moves
    set_1 = [
        create_move(
            movement_type="joint",
            joint_values=[0.0, -0.785, 0.785, 0.0, 1.57, 0.0],
            config=movement_configs["mid_move"]
        ),
        create_move(
            movement_type="joint",
            joint_values=[-0.175, -0.419, 1.378, 0.349, 1.535, -0.977],
            config=movement_configs["max_move"]
        ),
        create_move(
            movement_type="joint",
            joint_values=[0.733, -0.297, 1.378, -0.576, 1.692, 1.291],
            config=movement_configs["max_move"]
        )
    ]

    set_2 = [
        create_move(
            movement_type="named",
            named_target="home",
            config=movement_configs["mid_move"]
        ),
        create_move(
            movement_type="pose",
            target=Pose(
                position=Point(x=0.2, y=-0.1, z=0.2),
                orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
            ),
            config=movement_configs["mid_move"]
        ),
        create_move(
            movement_type="cartesian",
            target=Pose(
                position=Point(x=0.2, y=-0.1, z=0.15),
                orientation=Quaternion(x=1.0, y=0.0, z=0.0, w=0.0)
            ),
            config=movement_configs["slow_move"]
        ),
        create_move(
            movement_type="joint",
            joint_values=[0.0, -0.785, 0.785, 0.0, 1.57, 0.0],  # Back to rest
            config=movement_configs["mid_move"]
        )
    ]

    # --------------------------------------------------------------------------
    # 3) Create BlackBoard Keys
    # --------------------------------------------------------------------------
    blackboard = py_trees.blackboard.Blackboard()

    # Keys for Set 1
    planned_trajectory_keys_set1 = [f"planned_trajectory_set1_{i}" for i in range(len(set_1))]
    trajectory_valid_keys_set1 = [f"trajectory_valid_set1_{i}" for i in range(len(set_1))]

    # Keys for Set 2
    planned_trajectory_keys_set2 = [f"planned_trajectory_set2_{i}" for i in range(len(set_2))]
    trajectory_valid_keys_set2 = [f"trajectory_valid_set2_{i}" for i in range(len(set_2))]

    # --------------------------------------------------------------------------
    # 4) Create Planning and Execution Branches
    # --------------------------------------------------------------------------

    # PlanBranch_1
    plan_branch_1 = py_trees.composites.Sequence(name="PlanBranch_1", memory=True)
    for i, move in enumerate(set_1):
        move_goal = define_single_move_goal(move)
        plan_branch_1.add_child(
            PlanningBehaviour(
                name=f"PlanMoveSet1_{i}",
                goal=move_goal.goal,
                blackboard=blackboard,
                blackboard_key=planned_trajectory_keys_set1[i],
                validity_key=trajectory_valid_keys_set1[i],
                previous_key=planned_trajectory_keys_set1[i - 1] if i > 0 else None,
                node=node
            )
        )

    # ExecBranch_1
    exec_branch_1 = py_trees.composites.Sequence(name="ExecBranch_1", memory=True)
    for i in range(len(set_1)):
        exec_branch_1.add_child(
            ExecuteTrajectoryBehaviour(
                name=f"ExecuteMoveSet1_{i}",
                blackboard=blackboard,
                blackboard_key=planned_trajectory_keys_set1[i],
                validity_key=trajectory_valid_keys_set1[i],
                node=node
            )
        )

    # PlanBranch_2
    plan_branch_2 = py_trees.composites.Sequence(name="PlanBranch_2", memory=True)
    for i, move in enumerate(set_2):
        move_goal = define_single_move_goal(move)
        plan_branch_2.add_child(
            PlanningBehaviour(
                name=f"PlanMoveSet2_{i}",
                goal=move_goal.goal,
                blackboard=blackboard,
                blackboard_key=planned_trajectory_keys_set2[i],
                validity_key=trajectory_valid_keys_set2[i],
                previous_key=planned_trajectory_keys_set1[-1] if i == 0 else planned_trajectory_keys_set2[i - 1],
                node=node
            )
        )

    # ExecBranch_2
    exec_branch_2 = py_trees.composites.Sequence(name="ExecBranch_2", memory=True)
    for i in range(len(set_2)):
        exec_branch_2.add_child(
            ExecuteTrajectoryBehaviour(
                name=f"ExecuteMoveSet2_{i}",
                blackboard=blackboard,
                blackboard_key=planned_trajectory_keys_set2[i],
                validity_key=trajectory_valid_keys_set2[i],
                node=node
            )
        )

    # Parallel Node for ExecBranch_1 and PlanBranch_2
    parallel_exec_plan = py_trees.composites.Parallel(
        name="ExecBranch_1_and_PlanBranch_2",
        policy=py_trees.common.ParallelPolicy.SuccessOnAll()  # All children must succeed
    )
    parallel_exec_plan.add_child(exec_branch_1)
    parallel_exec_plan.add_child(plan_branch_2)

    # Sequence Node for ExecBranch_2 after Parallel Node
    exec_after_plan = py_trees.composites.Sequence(name="ExecAfterPlan", memory=True)
    exec_after_plan.add_child(parallel_exec_plan)
    exec_after_plan.add_child(exec_branch_2)

    # --------------------------------------------------------------------------
    # 5) Build the Behavior Tree
    # --------------------------------------------------------------------------
    root = py_trees.composites.Sequence(name="RootSequence", memory=True)
    root.add_child(plan_branch_1)  # Plan Set 1 first
    root.add_child(exec_after_plan)  # Execute Set 1 while planning Set 2, then Execute Set 2

    bt_tree = py_trees_ros.trees.BehaviourTree(root)

    # --------------------------------------------------------------------------
    # 6) Setup the tree
    # --------------------------------------------------------------------------
    try:
        bt_tree.setup(node=node, timeout=10.0)
    except Exception as e:
        node.get_logger().error(f"Failed to setup BehaviourTree: {e}")
        rclpy.shutdown()
        return

    # --------------------------------------------------------------------------
    # 7) Tick the tree until completion
    # --------------------------------------------------------------------------
    try:
        while rclpy.ok():
            bt_tree.tick()

            root_status = root.status
            if root_status == py_trees.common.Status.SUCCESS:
                node.get_logger().info("Behaviour Tree completed successfully.")
                break
            elif root_status == py_trees.common.Status.FAILURE:
                node.get_logger().error("Behaviour Tree failed.")
                break

            rclpy.spin_once(node, timeout_sec=0.01)
            time.sleep(0.01)

    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt, exiting...")

    # --------------------------------------------------------------------------
    # 8) Clean up
    # --------------------------------------------------------------------------
    bt_tree.shutdown()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
