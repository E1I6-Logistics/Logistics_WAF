"""Action server /pallet_replace (waffle_interfaces/action/PalletReplace).

Runs nav2_send_goal.replace_pallet for the goal's pick-up destination. Drop-off
(destination 2), second pick-up (destination 3) and return (destination 4) are fixed in
DESTINATIONS; the second pallet goes to the first pallet's aligned pose. The camera is
enabled through the Waffle camera manager right before each alignment and disabled
when that alignment ends (run_trial), and again when the mission ends; no OpenCV window.
"""
import threading

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from waffle_interfaces.action import PalletReplace
from nav2_send_goal import DESTINATIONS, ExistingPoseNavigator, replace_pallet
from pallet_alignment import set_camera
from pallet_alignment_trial import DEFAULT_CONFIG, validate_trial_config
from pallet_vision import load_config, validate_motion_config

NAV_TIMEOUT = 600.0


class PalletReplaceServer(Node):
    def __init__(self, cfg):
        super().__init__('pallet_replace_server')
        self.cfg = cfg
        self.busy = threading.Lock()
        self.stopping = False
        self.server = ActionServer(self, PalletReplace, 'pallet_replace', self.execute,
                                   goal_callback=self.accept,
                                   cancel_callback=lambda _: CancelResponse.ACCEPT,
                                   callback_group=ReentrantCallbackGroup())

    def accept(self, goal):
        if DESTINATIONS.get(goal.destination) is None:
            self.get_logger().warning(f'Rejected: destination {goal.destination} is not set')
            return GoalResponse.REJECT
        # One mission at a time; released when execute() finishes.
        if not self.busy.acquire(blocking=False):
            self.get_logger().warning('Rejected: a mission is already running')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def execute(self, goal_handle):
        feedback, result, nav = PalletReplace.Feedback(), PalletReplace.Result(), None

        def stage(name):
            self.get_logger().info(f'Stage: {name}')
            feedback.stage = name
            goal_handle.publish_feedback(feedback)

        try:
            try:
                nav = ExistingPoseNavigator()
                # Camera on (/camera/enable) right before alignment; show=False keeps the OpenCV window closed.
                result.success = replace_pallet(
                    nav, DESTINATIONS[goal_handle.request.destination], self.cfg, NAV_TIMEOUT,
                    show=False, enable_camera=True, stage=stage,
                    cancel=lambda: goal_handle.is_cancel_requested or self.stopping)
                result.message = 'Done' if result.success else f'{feedback.stage} failed'
            except RuntimeError as error:
                result.message = str(error)
            finally:
                if nav is not None:
                    try:
                        set_camera(nav, self.cfg['camera_enable_service'], False)
                    except RuntimeError as error:
                        self.get_logger().warning(f'Camera off failed: {error}')
                    finally:
                        nav.destroy_node()
            self.get_logger().info(f'Result: {result.message}')
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
            elif result.success:
                goal_handle.succeed()
            else:
                goal_handle.abort()
            return result
        finally:
            self.busy.release()


def main():
    # Validate before accepting any goal that could move the robot.
    cfg = load_config(DEFAULT_CONFIG)
    validate_trial_config(cfg)
    validate_motion_config(cfg)
    # Keep ROS alive during Ctrl+C cleanup so zero commands, Nav2 cancel and lift STOP can be sent.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    server = PalletReplaceServer(cfg)
    executor = MultiThreadedExecutor()
    executor.add_node(server)
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=.1)  # Bounded wait lets Ctrl+C raise here.
    except KeyboardInterrupt:
        server.stopping = True
        with server.busy:  # Wait for a running mission to stop the robot.
            pass
    finally:
        executor.shutdown()
        server.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
