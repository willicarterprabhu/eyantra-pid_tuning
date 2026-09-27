#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from swift_msgs.msg import SwiftMsgs

import time


class PythonAltitudeController(Node):

    def __init__(self):
        super().__init__('python_altitude_controller')

        # ============================================================
        # TARGET SETTINGS
        # ============================================================

        self.target_z = 3.0
        self.tolerance = 0.40

        # Required time to remain continuously inside ±0.4 m
        self.required_hold_time = 10.0

        # Maximum allowed time to reach target band
        self.settling_limit = 10.0

        # ============================================================
        # PID SETTINGS
        # ============================================================

        self.Kp = 55.0
        self.Ki = 0.0
        self.Kd = 70.0

        # ============================================================
        # THROTTLE SETTINGS
        # ============================================================

        self.base_throttle = 1519

        self.min_throttle = 1500
        self.max_throttle = 1525

        # ============================================================
        # STATE VARIABLES
        # ============================================================

        self.current_z = None

        self.previous_error = None
        self.integral = 0.0
        self.previous_time = None

        # Timer used for complete test
        self.start_time = None

        # Time when drone first enters ±0.4 m band
        self.target_reached_time = None

        # Used for continuous 10-second hold
        self.band_start_time = None

        # ============================================================
        # MEASUREMENTS
        # ============================================================

        self.min_z = float('inf')
        self.max_z = float('-inf')

        self.max_abs_error = 0.0

        self.test_finished = False
        self.target_reached_within_10s = False
        self.hold_requirement_passed = False

        # ============================================================
        # ROS SUBSCRIBER
        # ============================================================

        self.odom_sub = self.create_subscription(
            Odometry,
            '/rotors/odometry',
            self.odometry_callback,
            10
        )

        # ============================================================
        # ROS PUBLISHER
        # ============================================================

        self.command_pub = self.create_publisher(
            SwiftMsgs,
            '/drone_command',
            10
        )

        # ============================================================
        # CONTROL LOOP
        # ============================================================

        self.control_timer = self.create_timer(
            0.05,          # 20 Hz
            self.control_loop
        )

        self.get_logger().info(
            "=============================================="
        )

        self.get_logger().info(
            "Python Altitude PID Controller Started"
        )

        self.get_logger().info(
            f"Target altitude       : {self.target_z:.2f} m"
        )

        self.get_logger().info(
            f"Allowed error         : ±{self.tolerance:.2f} m"
        )

        self.get_logger().info(
            f"Allowed altitude     : "
            f"{self.target_z - self.tolerance:.2f} - "
            f"{self.target_z + self.tolerance:.2f} m"
        )

        self.get_logger().info(
            f"Settling limit        : "
            f"{self.settling_limit:.1f} seconds"
        )

        self.get_logger().info(
            f"Required hold time    : "
            f"{self.required_hold_time:.1f} seconds"
        )

        self.get_logger().info(
            f"Kp={self.Kp}  Ki={self.Ki}  Kd={self.Kd}"
        )

        self.get_logger().info(
            "=============================================="
        )

    # ================================================================
    # ODOMETRY CALLBACK
    # ================================================================

    def odometry_callback(self, msg):

        self.current_z = msg.pose.pose.position.z

        # Start the test timer when the first altitude
        # measurement is received.
        if self.start_time is None:

            self.start_time = time.monotonic()

            self.get_logger().info(
                "Altitude measurement received."
            )

            self.get_logger().info(
                "Test timer STARTED."
            )

    # ================================================================
    # SEND DRONE COMMAND
    # ================================================================

    def publish_command(self, throttle):

        msg = SwiftMsgs()

        msg.rc_roll = 1500
        msg.rc_pitch = 1500
        msg.rc_yaw = 1500

        msg.rc_throttle = int(throttle)

        msg.rc_aux1 = 0
        msg.rc_aux2 = 0
        msg.rc_aux3 = 0

        # ARM
        msg.rc_aux4 = 2000

        msg.drone_index = 0

        self.command_pub.publish(msg)

    # ================================================================
    # SAFE STOP
    # ================================================================

    def safe_stop(self):

        if not rclpy.ok():
            return

        self.get_logger().info(
            "Sending neutral throttle..."
        )

        # Send several neutral commands
        # to make sure the drone receives the stop command.

        for _ in range(10):

            if not rclpy.ok():
                break

            self.publish_command(1500)

            time.sleep(0.02)

    # ================================================================
    # MAIN PID CONTROL LOOP
    # ================================================================

    def control_loop(self):

        if self.current_z is None:
            return

        if self.test_finished:
            return

        # ------------------------------------------------------------
        # ELAPSED TEST TIME
        # ------------------------------------------------------------

        elapsed_time = time.monotonic() - self.start_time

        # ------------------------------------------------------------
        # ALTITUDE ERROR
        # ------------------------------------------------------------

        error = self.target_z - self.current_z

        abs_error = abs(error)

        # ------------------------------------------------------------
        # MEASUREMENTS
        # ------------------------------------------------------------

        if self.current_z < self.min_z:
            self.min_z = self.current_z

        if self.current_z > self.max_z:
            self.max_z = self.current_z

        if abs_error > self.max_abs_error:
            self.max_abs_error = abs_error

        # ------------------------------------------------------------
        # PID TIME
        # ------------------------------------------------------------

        now = time.monotonic()

        if self.previous_time is None:

            dt = 0.05

        else:

            dt = now - self.previous_time

            if dt <= 0.0:
                dt = 0.05

        self.previous_time = now

        # ------------------------------------------------------------
        # INTEGRAL
        # ------------------------------------------------------------

        self.integral += error * dt

        # Anti-windup
        if self.integral > 0.5:
            self.integral = 0.5

        if self.integral < -0.5:
            self.integral = -0.5

        # ------------------------------------------------------------
        # DERIVATIVE
        # ------------------------------------------------------------

        if self.previous_error is None:

            derivative = 0.0

        else:

            derivative = (error - self.previous_error) / dt

        self.previous_error = error

        # ------------------------------------------------------------
        # PID OUTPUT
        # ------------------------------------------------------------

        pid_output = (
            self.Kp * error
            + self.Ki * self.integral
            + self.Kd * derivative
        )

        # ------------------------------------------------------------
        # THROTTLE
        # ------------------------------------------------------------

        throttle = self.base_throttle + pid_output

        # Clamp throttle
        throttle = max(
            self.min_throttle,
            min(self.max_throttle, throttle)
        )

        throttle = int(throttle)

        # ------------------------------------------------------------
        # TARGET BAND CHECK
        # ------------------------------------------------------------

        inside_band = abs_error <= self.tolerance

        # ------------------------------------------------------------
        # FIRST TARGET BAND ENTRY
        # ------------------------------------------------------------

        if inside_band:

            if self.target_reached_time is None:

                self.target_reached_time = elapsed_time

                if elapsed_time <= self.settling_limit:

                    self.target_reached_within_10s = True

                    self.get_logger().info(
                        "=============================================="
                    )

                    self.get_logger().info(
                        f"TARGET BAND REACHED at "
                        f"{elapsed_time:.2f} seconds"
                    )

                    self.get_logger().info(
                        f"Z={self.current_z:.3f} m"
                    )

                    self.get_logger().info(
                        f"Error={error:.3f} m"
                    )

                    self.get_logger().info(
                        "Reached within 10-second limit: YES"
                    )

                    self.get_logger().info(
                        "Starting 10-second continuous hold timer..."
                    )

                    self.get_logger().info(
                        "=============================================="
                    )

                else:

                    self.target_reached_within_10s = False

                    self.get_logger().warn(
                        f"TARGET BAND REACHED at "
                        f"{elapsed_time:.2f} seconds"
                    )

                    self.get_logger().warn(
                        "Target was NOT reached within 10 seconds."
                    )

            # --------------------------------------------------------
            # START CONTINUOUS HOLD TIMER
            # --------------------------------------------------------

            if self.band_start_time is None:

                self.band_start_time = time.monotonic()

        else:

            # Drone left the allowed ±0.4 m band.
            # Reset the continuous hold timer.

            if self.band_start_time is not None:

                self.get_logger().warn(
                    "Drone left target band. "
                    "10-second hold timer RESET."
                )

            self.band_start_time = None

        # ------------------------------------------------------------
        # CHECK 10-SECOND CONTINUOUS HOLD
        # ------------------------------------------------------------

        if self.band_start_time is not None:

            band_time = (
                time.monotonic() - self.band_start_time
            )

            if band_time >= self.required_hold_time:

                self.hold_requirement_passed = True
                self.test_finished = True

                self.get_logger().info(
                    "=============================================="
                )

                self.get_logger().info(
                    "TASK 1B REQUIREMENT PASSED"
                )

                self.get_logger().info(
                    "Drone stayed inside ±0.4 m "
                    "for 10 continuous seconds."
                )

                self.get_logger().info(
                    f"Total test time = "
                    f"{elapsed_time:.2f} seconds"
                )

                self.get_logger().info(
                    "=============================================="
                )

                self.safe_stop()

                self.print_final_report()

                rclpy.shutdown()

                return

        # ------------------------------------------------------------
        # PRINT LIVE STATUS
        # ------------------------------------------------------------

        self.get_logger().info(
            f"Time={elapsed_time:.2f}s "
            f"Z={self.current_z:.3f} "
            f"Error={error:.3f} "
            f"Throttle={throttle}"
        )

        # ------------------------------------------------------------
        # 10-SECOND REACH CHECK
        # ------------------------------------------------------------

        if (
            elapsed_time > self.settling_limit
            and self.target_reached_time is None
        ):

            self.get_logger().warn(
                "10 seconds exceeded without reaching "
                "the ±0.4 m target band."
            )

        # ------------------------------------------------------------
        # PUBLISH COMMAND
        # ------------------------------------------------------------

        self.publish_command(throttle)

    # ================================================================
    # FINAL REPORT
    # ================================================================

    def print_final_report(self):

        total_time = 0.0

        if self.start_time is not None:

            total_time = (
                time.monotonic() - self.start_time
            )

        print("")
        print("==============================================")
        print("           TASK 1B FINAL REPORT")
        print("==============================================")

        print(
            f"Setpoint              : "
            f"{self.target_z:.3f} m"
        )

        print(
            f"Allowed error         : "
            f"±{self.tolerance:.3f} m"
        )

        print(
            f"Allowed altitude      : "
            f"{self.target_z - self.tolerance:.3f} "
            f"to "
            f"{self.target_z + self.tolerance:.3f} m"
        )

        print(
            f"Minimum altitude      : "
            f"{self.min_z:.3f} m"
        )

        print(
            f"Maximum altitude      : "
            f"{self.max_z:.3f} m"
        )

        print(
            f"Maximum absolute error: "
            f"{self.max_abs_error:.3f} m"
        )

        print(
            f"Total test time       : "
            f"{total_time:.2f} s"
        )

        # ------------------------------------------------------------
        # TARGET REACHED TIME
        # ------------------------------------------------------------

        if self.target_reached_time is not None:

            print(
                f"Target band reached   : "
                f"{self.target_reached_time:.2f} s"
            )

            if self.target_reached_time <= self.settling_limit:

                print(
                    "10-second reach limit : PASSED"
                )

            else:

                print(
                    "10-second reach limit : FAILED"
                )

        else:

            print(
                "Target band reached   : NOT REACHED"
            )

            print(
                "10-second reach limit : FAILED"
            )

        # ------------------------------------------------------------
        # CONTINUOUS HOLD
        # ------------------------------------------------------------

        if self.hold_requirement_passed:

            print(
                "10-second continuous  : PASSED"
            )

        else:

            print(
                "10-second continuous  : FAILED"
            )

        # ------------------------------------------------------------
        # OVERALL
        # ------------------------------------------------------------

        if (
            self.target_reached_within_10s
            and self.hold_requirement_passed
        ):

            print("")
            print(
                ">>> TASK 1B REQUIREMENT PASSED <<<"
            )

        else:

            print("")
            print(
                ">>> TASK 1B REQUIREMENT NOT PASSED <<<"
            )

        print("==============================================")
        print("")


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = PythonAltitudeController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            "Keyboard interrupt received."
        )

    except Exception as e:

        node.get_logger().error(
            f"Controller error: {e}"
        )

    finally:

        # Only attempt ROS operations while ROS is alive
        if rclpy.ok():

            node.safe_stop()

            node.print_final_report()

            node.destroy_node()

            rclpy.shutdown()


if __name__ == '__main__':

    main()