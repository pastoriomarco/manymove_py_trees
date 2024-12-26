# src/manymove_py_trees/manymove_py_trees/tree_creation_helpers.py

import py_trees
import py_trees_ros
from manymove_py_trees.move_definitions import define_single_move_goal, Move
from manymove_py_trees.planning_behavior import PlanningBehaviour
from manymove_py_trees.execution_behavior import ExecuteTrajectoryBehaviour
from typing import List

def create_tree_from_sequences(
    node,
    list_of_move_sequences: List[List[Move]],
    root_name: str = "RootSequence"
) -> py_trees_ros.trees.BehaviourTree:
    """
    Creates a Behavior Tree from a list of move-sequences.

    :param node: The ROS 2 node.
    :param list_of_move_sequences: A list where each element is a list of `Move` objects.
                                   Example:
                                     [
                                         scan_surroundings,
                                         pick_sequence,
                                         rest_and_home
                                     ]
    :param root_name: Name of the root behavior.
    :return: A py_trees_ros.trees.BehaviourTree instance fully set up.
    """
    # Create a blackboard
    blackboard = py_trees.blackboard.Blackboard()

    # Create the root Sequence
    root = py_trees.composites.Sequence(name=root_name, memory=True)

    previous_sequence_last_planned_key = None  # To chain sequences if needed

    for seq_index, move_sequence in enumerate(list_of_move_sequences):
        # Generate unique Blackboard Keys for the current sequence
        planned_keys = [
            f"planned_trajectory_seq{seq_index}_move{i}"
            for i in range(len(move_sequence))
        ]
        validity_keys = [
            f"valid_trajectory_seq{seq_index}_move{i}"
            for i in range(len(move_sequence))
        ]

        # Create a Planning branch for the current sequence
        plan_branch = py_trees.composites.Sequence(name=f"PlanSequence_{seq_index}", memory=True)
        for i, move in enumerate(move_sequence):
            # Convert the Move object into a single MoveManipulator.Goal
            move_goal = define_single_move_goal(move)

            # Determine the previous_key for chaining
            if i == 0 and previous_sequence_last_planned_key is not None:
                previous_key = previous_sequence_last_planned_key
            elif i > 0:
                previous_key = planned_keys[i - 1]
            else:
                previous_key = None

            plan_beh = PlanningBehaviour(
                name=f"Plan_Seq{seq_index}_Move{i}",
                goal=move_goal.goal,
                blackboard=blackboard,
                blackboard_key=planned_keys[i],
                validity_key=validity_keys[i],
                previous_key=previous_key,
                node=node
            )
            plan_branch.add_child(plan_beh)

        # Create an Execution branch for the current sequence
        exec_branch = py_trees.composites.Sequence(name=f"ExecSequence_{seq_index}", memory=True)
        for i in range(len(move_sequence)):
            exec_beh = ExecuteTrajectoryBehaviour(
                name=f"Execute_Seq{seq_index}_Move{i}",
                blackboard=blackboard,
                blackboard_key=planned_keys[i],
                validity_key=validity_keys[i],
                node=node
            )
            exec_branch.add_child(exec_beh)

        # Add Planning and Execution branches to the root in sequence
        root.add_child(plan_branch)
        root.add_child(exec_branch)

        # Update the last planned key to the last move of the current sequence
        if planned_keys:
            previous_sequence_last_planned_key = planned_keys[-1]

    # Finally, create the ROS Behavior Tree
    bt_tree = py_trees_ros.trees.BehaviourTree(root)

    return bt_tree
