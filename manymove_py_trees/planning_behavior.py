import py_trees
from manymove_planner.srv import PlanManipulator
from rclpy.node import Node
import rclpy

class PlanningBehaviour(py_trees.behaviour.Behaviour):
    """
    Behavior to call the planning service and store the result in the Blackboard.
    """

    def __init__(self, name, goal, blackboard, blackboard_key: str, validity_key: str, previous_key: str = None, node: Node = None):
        super().__init__(name)
        self.goal = goal
        self.blackboard = blackboard
        self.blackboard_key = blackboard_key
        self.validity_key = validity_key
        self.previous_key = previous_key
        self.node = node
        self.client = node.create_client(PlanManipulator, 'plan_manipulator')
        self.planning_done = False
        self.planning_success = False

    def initialise(self):
        self.node.get_logger().info(f"[{self.name}] Initializing PlanningBehaviour.")
        self.planning_done = False
        self.planning_success = False

    def update(self):
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().error(f"[{self.name}] Planning service 'plan_manipulator' not available.")
            return py_trees.common.Status.FAILURE

        if not self.planning_done:
            # Use the previous key for start_joint_values if provided
            if self.previous_key:
                previous_trajectory = self.blackboard.get(self.previous_key)

                if not previous_trajectory:
                    self.node.get_logger().error(f"[{self.name}] Previous trajectory is missing or invalid.")
                    return py_trees.common.Status.FAILURE

                # Update the start_joint_values in the goal
                self.goal.start_joint_values = previous_trajectory.joint_trajectory.points[-1].positions

            # Call the planning service
            request = PlanManipulator.Request()
            request.goal = self.goal
            self.node.get_logger().info(f"[{self.name}] Sending planning request to 'plan_manipulator'.")
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future)
            response = future.result()

            if response and response.success:
                self.node.get_logger().info(f"[{self.name}] Planning succeeded.")
                self.blackboard.set(self.blackboard_key, response.trajectory)
                self.blackboard.set(self.validity_key, True)  # Mark trajectory as valid
                self.planning_success = True
            else:
                self.node.get_logger().error(f"[{self.name}] Planning failed.")
                self.planning_success = False

            self.planning_done = True

        return py_trees.common.Status.SUCCESS if self.planning_success else py_trees.common.Status.FAILURE

