import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tello_msgs.srv import TelloAction

import numpy as np
import paho.mqtt.client as mqtt


class ActionManager(Node):
    def __init__(self):
        super().__init__('controller_manager')
        self.cli = self.create_client(TelloAction, '/drone1/tello_action')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = TelloAction.Request()

    def ask_for_takeoff(self):
        self.req.cmd = "takeoff"
        self.future = self.cli.call_async(self.req)

    def ask_for_landing(self):
        self.req.cmd = "land"
        self.future = self.cli.call_async(self.req)


class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        # CREATE THE PUBLISHER FOR COMMAND VELOCITY
        self.cmd_vel_publisher_ = self.create_publisher(
            Twist, '/drone1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.cmd_vel_loop)
        # Init desired speed
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.v_yaw = 0.0

    def cmd_vel_loop(self):
        # TODO : execute received command safely
        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.linear.z = self.vz
        msg.angular.z = self.v_yaw

        # Reset values
        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0
        self.v_yaw = 0.0

        # Publishing
        self.cmd_vel_publisher_.publish(msg)

    def set_commands(self, vx=None, vy=None, vz=None, v_yaw=None):
        if vx is not None:
            self.vx = float(vx)
        if vy is not None:
            self.vy = float(vy)
        if vz is not None:
            self.vz = float(vz)
        if v_yaw is not None:
            self.v_yaw = float(v_yaw)


# Global variables

broker = "mqtt.eclipseprojects.io"
broker_port = 1883

airborn = False
flag_service_takeoff_called = False
flag_service_landing_called = False


def on_message(client, userdata, msg):

    global flag_service_takeoff_called, airborn, flag_service_landing_called
    topic = msg.topic
    topic = topic.split('/')[-1]
    speed = 0.5

    if topic == "takeoff":
        takeoff = int(str(msg.payload.decode("utf-8")))
        if takeoff == 1 and airborn is False:
            flag_service_takeoff_called = True
            action_manager.ask_for_takeoff()

    if topic == "landing" and airborn:
        action_manager.ask_for_landing()
        flag_service_landing_called = True

    if topic == "vx" and airborn is True:
        vx = int(str(msg.payload.decode("utf-8")))
        controller.vx = vx * speed
        print("vx")

    if topic == "vy" and airborn is True:
        vy = int(str(msg.payload.decode("utf-8")))
        controller.vy = vy * speed
        print("vy")

    if topic == "vz" and airborn is True:
        vz = int(str(msg.payload.decode("utf-8")))
        controller.vz = vz * speed
        print("vz")

    if topic == "v_yaw" and airborn is True:
        v_yaw = int(str(msg.payload.decode("utf-8")))
        controller.v_yaw = v_yaw * speed
        print("v_yaw")


def main(args=None):
    global action_manager, controller, airborn

    rclpy.init(args=args)

    # # Wait for take off service to be ready
    action_manager = ActionManager()
    controller = Controller()

    # MQTT initialisation
    client = mqtt.Client("Drone")
    client.connect(broker, broker_port)
    client.subscribe("YONDU/DroneCommand/#")
    client.on_message = on_message
    client.loop_start()

    # TODO : this action should be triggered by MQTT communication
    # action_manager.ask_for_takeoff()

    ready_to_continue_mission = False

    # Try to takeoff, wait for the return of the service
    while rclpy.ok():
        rclpy.spin_once(action_manager)
        if action_manager.future.done():
            try:
                response = action_manager.future.result()
                if response.rc == 1:  # OK
                    print("Take off is a success !")
                    airborn = True
                    ready_to_continue_mission = True
                else:  # NOT OK
                    print("Something is wrong with the takeoff. LANDING NOW")
            except Exception as e:
                action_manager.get_logger().info("Service call failed %r" % (e,))
            break

    if ready_to_continue_mission:
        try:
            while rclpy.ok():
                rclpy.spin_once(controller)
                if flag_service_landing_called:
                    break
        except KeyboardInterrupt:
            print("Stopping the control. Ask for landing.")
            action_manager.ask_for_landing()
        controller.destroy_node()

    while rclpy.ok():
        rclpy.spin_once(action_manager)
        if action_manager.future.done():
            try:
                response = action_manager.future.result()
                if response.rc == 1:  # OK
                    print("Landing is a success !")
                    airborn = False
                    break  # Only if landing is ok
            except Exception as e:
                action_manager.get_logger().info("Service call failed %r" % (e,))
            break

    client.loop_stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
