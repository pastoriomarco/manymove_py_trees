# src/manymove_py_trees/manymove_py_trees/tree_helper.py

import py_trees
import py_trees_ros
from manymove_py_trees.move_definitions import define_single_move_goal, Move
from manymove_py_trees.planning_behavior import PlanningActionBehaviour
from manymove_py_trees.execution_behavior import ExecuteTrajectoryBehaviour
from typing import List

# This helper function parallelizes the planning and the execution of a series of moves
# The sequences will be treated as single moves for planning and execution, instead of planning to obtain a single trajectory to be then executed:
# The reason for this is to be able in the future to intercept a failed execution and to replan only the last move, and not the whole trajectory
# Replanning the whole trajectory would be simpler, but the execution from a random point to the first of the sequence may pose problems
# Instead, replanning from the current position to the point that was previously requested is more coherent with the intended behavior
def create_tree_from_sequences(
    node,
    list_of_move_sequences: List[List[Move]],
    root_name: str = "RootSequence"
) -> py_trees_ros.trees.BehaviourTree:
    """
    Creates a Behavior Tree that matches the "parallel plan & execute" logic
    seen in the old version:
    
      1) Plan the first sequence fully.
      2) Parallel: Exec the first sequence, Plan the second.
      3) Parallel: Exec the second sequence, Plan the third.
      ...
      N) Finally, execute the last sequence alone.

    This replicates the pattern:
        RootSequence
        ├─ PlanSequence_0
        ├─ Parallel( ExecSequence_0, PlanSequence_1 )
        ├─ Parallel( ExecSequence_1, PlanSequence_2 )
        ├─ ...
        └─ ExecSequence_(n-1)
    """

    # Shared blackboard
    blackboard = py_trees.blackboard.Blackboard()

    # We'll store planning and execution branches for each sequence
    plan_branches = []
    exec_branches = []

    # For chaining: store the last trajectory key of each sequence
    # so that the next sequence’s first move can start from that final position.
    last_planned_keys_per_seq = []

    # ---------------------------------------------------------------
    # 1) Create all plan/exec branches for each sequence
    # ---------------------------------------------------------------
    for seq_index, moves in enumerate(list_of_move_sequences):
        # Unique blackboard keys for each move in the sequence
        planned_keys = [
            f"planned_traj_seq{seq_index}_move{i}" for i in range(len(moves))
        ]
        validity_keys = [
            f"valid_traj_seq{seq_index}_move{i}" for i in range(len(moves))
        ]

        # Create planning branch for this sequence
        plan_branch = py_trees.composites.Sequence(
            name=f"PlanSequence_{seq_index}", memory=True
        )
        for i, move in enumerate(moves):
            single_goal = define_single_move_goal(move).goal

            # For the first move of *this* sequence, chain from the last move
            # of the *previous* sequence if available, otherwise from the
            # previous move in the same sequence.
            if i == 0 and seq_index > 0:
                # chain from the last planned key of the previous sequence
                previous_key = last_planned_keys_per_seq[seq_index - 1]
            elif i > 0:
                # chain from the previous move in the same sequence
                previous_key = planned_keys[i - 1]
            else:
                previous_key = None

            plan_behavior = PlanningActionBehaviour(
                name=f"PlanMove_{seq_index}_{i}",
                goal=single_goal,
                blackboard=blackboard,
                blackboard_key=planned_keys[i],
                validity_key=validity_keys[i],
                previous_key=previous_key,
                node=node
            )
            plan_branch.add_child(plan_behavior)

        # Create execution branch for this sequence
        exec_branch = py_trees.composites.Sequence(
            name=f"ExecSequence_{seq_index}", memory=True
        )
        for i in range(len(moves)):
            exec_behavior = ExecuteTrajectoryBehaviour(
                name=f"ExecuteMove_{seq_index}_{i}",
                blackboard=blackboard,
                blackboard_key=planned_keys[i],
                validity_key=validity_keys[i],
                node=node
            )
            exec_branch.add_child(exec_behavior)

        plan_branches.append(plan_branch)
        exec_branches.append(exec_branch)

        # The last planned key of this sequence is the final entry in planned_keys
        if len(planned_keys) > 0:
            last_planned_keys_per_seq.append(planned_keys[-1])
        else:
            last_planned_keys_per_seq.append(None)  # Shouldn't happen if there are moves

    # ---------------------------------------------------------------
    # 2) Build the root logic to replicate the old "Parallel" pattern
    # ---------------------------------------------------------------
    root = py_trees.composites.Sequence(name=root_name, memory=True)

    n = len(list_of_move_sequences)
    if n == 0:
        # No sequences to run, just return an empty tree
        return py_trees_ros.trees.BehaviourTree(root)

    # a) First, plan the 0th sequence
    root.add_child(plan_branches[0])

    # b) For each subsequent sequence i in [1..n-1],
    #    add a Parallel( exec seq i-1, plan seq i )
    #    so that we execute the previous sequence while planning the next
    for seq_index in range(1, n):
        parallel_node = py_trees.composites.Parallel(
            name=f"Parallel_Exec{seq_index-1}_Plan{seq_index}",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll()
        )
        # Exec the previous sequence
        parallel_node.add_child(exec_branches[seq_index - 1])
        # Plan the current sequence
        parallel_node.add_child(plan_branches[seq_index])

        root.add_child(parallel_node)

    # c) Finally, execute the *last* sequence
    root.add_child(exec_branches[n - 1])

    # Wrap in a ROS BehaviorTree
    return py_trees_ros.trees.BehaviourTree(root)
