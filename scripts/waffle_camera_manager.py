"""Run on Waffle with camera_ws sourced; manage only the child we started."""
import argparse
from pathlib import Path
import signal
import subprocess

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class CameraManager(Node):
    def __init__(self, params_file):
        super().__init__('waffle_camera_manager')
        self.params_file = Path(params_file).resolve()
        if not self.params_file.is_file():
            raise ValueError(f'Camera parameters not found: {self.params_file}')
        self.binary = Path(get_package_prefix('camera_ros')) / 'lib/camera_ros/camera_node'
        if not self.binary.is_file():
            raise ValueError(f'Camera executable not found: {self.binary}')
        self.process = None
        self.create_service(SetBool, '/camera/enable', self.enable)
        self.get_logger().info('Ready: /camera/enable. Camera starts only when requested.')

    def stop_owned(self):
        if self.process is None or self.process.poll() is not None:
            self.process = None
            return
        self.process.send_signal(signal.SIGINT)
        try:
            self.process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            raise RuntimeError(f'Owned camera PID {self.process.pid} did not stop; inspect before restarting')
        self.process = None

    def enable(self, request, response):
        try:
            if request.data:
                if self.process is not None and self.process.poll() is None:
                    response.message = f'Owned camera already running: PID {self.process.pid}'
                elif self.count_publishers('/camera/image_raw/compressed') > 0:
                    response.message = 'Reusing existing camera publisher; manager does not own it'
                else:
                    command = [str(self.binary), '--ros-args', '--params-file', str(self.params_file)]
                    self.get_logger().info(f'Waffle subprocess (no shell): {command}')
                    self.process = subprocess.Popen(command)
                    response.message = f'Start requested: PID {self.process.pid}; client must verify fresh images'
            else:
                self.stop_owned()
                response.message = 'Owned camera stopped; external camera processes left untouched'
            response.success = True
        except (OSError, RuntimeError) as error:
            response.success = False
            response.message = str(error)
        return response


def main():
    parser = argparse.ArgumentParser(description='Waffle camera start/stop ROS service')
    parser.add_argument('--params-file', type=Path,
                        default=Path.home()/'camera_ws/config/waffle_camera.yaml')
    args = parser.parse_args()
    rclpy.init()
    node = None
    try:
        node = CameraManager(args.params_file)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.stop_owned()
            finally:
                node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
