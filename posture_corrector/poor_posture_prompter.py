import numpy as np
import subprocess
from parse import parse


import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration

from sensor_msgs.msg import Imu
from example_interfaces.msg import Bool

def vector_distance(v1:np.array, v2:np.array):
    return np.linalg.norm(v1-v2)

class Debouncer:
    """
    Responsible for debouncing posture good/bad readings
    """

    def __init__(self, initial_state:bool, current_time:Time, debounce_time:Duration) -> None:
        self.state=initial_state
        self._noisy_state = initial_state
        self._last_switch_time = current_time
        self._debounce_time = debounce_time

    def update(self, state:bool, current_time:Time):
        if self._noisy_state != state:
            # state switch
            self._noisy_state = state
            self._last_switch_time = current_time

        time_since_last_state_switch = current_time - self._last_switch_time
        if time_since_last_state_switch > self._debounce_time:
            self.state = self._noisy_state

class PostureNotificationManager:
    """
    Sends notifcations when posture goes from 
    good to bad
    Setting and clearing are based on:
    https://superuser.com/questions/1592674/how-to-clear-notification-sent-by-shell-script
    """
    def __init__(self):
        self.notification_id = None
        self.notification_header = "Sit Up!"
        self.notification_body = "Slouching detected"

    def update(self, posture_is_good:bool):
        if posture_is_good and self.notification_id is not None:
            self.clear_notification()

        elif not posture_is_good and self.notification_id is None:
            self.send_notification()

        self.posture_is_good = posture_is_good

    def send_notification(self):
        send_cmd = [
            "gdbus", "call",
            "--session",
            "--dest", "org.freedesktop.Notifications",
            "--object-path", "/org/freedesktop/Notifications",
            "--method", "org.freedesktop.Notifications.Notify",
            "posture_notifier",
            "0",
            "face-tired",
            f'"{self.notification_header}"',
            f'"{self.notification_body}"',
            "[]",
            "{}",
            "0"]
        # print(" ".join(send_cmd))
        result = subprocess.run(send_cmd, capture_output=True)

        stdout = result.stdout.decode('ascii').strip()
        self.notification_id = parse("(uint32 {id:d},)", stdout)['id']

    def clear_notification(self):
        subprocess.run([
            "gdbus", "call",
            "--session",
            "--dest", "org.freedesktop.Notifications",
            "--object-path", "/org/freedesktop/Notifications",
            "--method", "org.freedesktop.Notifications.CloseNotification",
            str(self.notification_id),
        ], capture_output=True)
        self.notification_id = None


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('poor_posture_prompter')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('good_accel', [0.0,0.0,9.8]),
                ('bad_accel', [9.8,0.0,0.0]),
                ('debounce_time_sec', 1),
                ('print_imu_vec', False),
                ('notify_bad_posture', True),
            ]
        )

        (good_accel_param, bad_accel_param, print_imu_vec_param) = self.get_parameters(
            ['good_accel', 'bad_accel', 'print_imu_vec'])
        self.good_accel = good_accel_param.value
        self.bad_accel = bad_accel_param.value
        self.print_imu_vec = print_imu_vec_param.value

        if self.get_parameter("notify_bad_posture").value:
            self.posture_notifier = PostureNotificationManager()
        else:
            self.posture_notifier = None

        self.posture_is_good = True # assume the best in people by default
        self.posture_debouncer = Debouncer(
            initial_state=self.posture_is_good,
            current_time=self.get_clock().now(),
            debounce_time=Duration(
                seconds = self.get_parameter("debounce_time_sec").value
            ))

        self.publisher = self.create_publisher(Bool, 'posture_is_good', 10)

        self.subscription = self.create_subscription(
            Imu,
            'imu/data_raw',
            self.imu_callback,
            10)
        self.subscription  # prevent unused variable warning


    def imu_callback(self, msg:Imu):
        imu_vec = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        good_dist = vector_distance(self.good_accel, imu_vec)
        bad_dist = vector_distance(self.bad_accel, imu_vec)
        # self.get_logger().info(f"{imu_vec=}")
        # self.get_logger().info(f"{self.good_accel=}")
        # self.get_logger().info(f"{self.bad_accel=}")
        # self.get_logger().info(f"{good_dist=}")
        # self.get_logger().info(f"{bad_dist=}")

        posture_is_good = good_dist < bad_dist

        self.posture_debouncer.update(posture_is_good, self.get_clock().now())
        self.posture_is_good = self.posture_debouncer.state

        if self.print_imu_vec:
            self.get_logger().info(f"{imu_vec=}")

        if self.posture_notifier is not None:
            self.posture_notifier.update(self.posture_is_good)

        self.publisher.publish(Bool(data=bool(self.posture_is_good)))




def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()