import rclpy
from nav2_msgs.action import DockRobot, NavigateToPose
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor

from mission.stub_action_servers import StubActionServers


def _call_action_and_wait(executor, client, goal, timeout_sec=5.0):
    send_future = client.send_goal_async(goal)
    executor.spin_until_future_complete(send_future, timeout_sec=timeout_sec)
    goal_handle = send_future.result()
    assert goal_handle.accepted
    result_future = goal_handle.get_result_async()
    executor.spin_until_future_complete(result_future, timeout_sec=timeout_sec)
    return result_future.result()


def test_navigate_to_pose_succeeds():
    rclpy.init()
    try:
        server = StubActionServers(delay_sec=0.05)
        client_node = rclpy.create_node("test_client")
        client = ActionClient(client_node, NavigateToPose, "navigate_to_pose")
        executor = SingleThreadedExecutor()
        executor.add_node(server)
        executor.add_node(client_node)

        assert client.wait_for_server(timeout_sec=5.0)
        result = _call_action_and_wait(executor, client, NavigateToPose.Goal())

        assert result.status == 4  # GoalStatus.STATUS_SUCCEEDED
    finally:
        rclpy.shutdown()


def test_dock_robot_succeeds():
    rclpy.init()
    try:
        server = StubActionServers(delay_sec=0.05)
        client_node = rclpy.create_node("test_client")
        client = ActionClient(client_node, DockRobot, "dock_robot")
        executor = SingleThreadedExecutor()
        executor.add_node(server)
        executor.add_node(client_node)

        assert client.wait_for_server(timeout_sec=5.0)
        result = _call_action_and_wait(executor, client, DockRobot.Goal())

        assert result.status == 4  # GoalStatus.STATUS_SUCCEEDED
    finally:
        rclpy.shutdown()
