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
        self.cmd_vel_publisher_.publish(msg)

    def set_commands(self,vx,vy,vz, v_yaw):
        self.vx = float(vx)
        self.vy = float(vy)
        self.vz = float(vz)
        self.v_yaw = float(v_yaw)


# Global variables

broker = "mqtt.eclipseprojects.io"
broker_port = 1883
topic_name = "takeoff"

airborn = False
flag_service_takeoff_called = False


def on_message(client, userdata, msg):
    global received_msg
    topic = msg.topic

    if topic == "takeoff":
        takeoff = int(str(msg.payload.decode("utf-8")))
        if takeoff == 1 and airborn is False:
            flag_service_takeoff_called = True
            action_manager.ask_for_takeoff()
            

    msg_decode = str(msg.payload.decode("utf-8"))
    print("Message received:", msg_decode, "on topic", topic)
    received_msg = msg_decode


def main(args=None):
    global action_manager, controller

    rclpy.init(args=args)



    # # Wait for take off service to be ready
    action_manager = ActionManager()
    controller = Controller()

    # MQTT initialisation
    client = mqtt.Client("Drone")
    client.connect(broker, broker_port)
    client.subscribe(topic_name)
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
        # controller = Controller()
        try:
            while rclpy.ok():
                rclpy.spin_once(controller)
                # TODO : listen to the server to see if new information are available and set the command on the controller
                # TODO : stop the drone if landing is asked
        except KeyboardInterrupt:
            print("Stopping the control. Ask for landing.")
        controller.destroy_node()

    # ASK FOR LANDING
    # TODO : maybe change the location of this command
    action_manager.ask_for_landing()
    while rclpy.ok():
        rclpy.spin_once(action_manager)
        if action_manager.future.done():
            try:
                response = action_manager.future.result()
                if response.rc == 1:  # OK
                    print("Landing is a success !")
                    break  # Only if landing is ok
            except Exception as e:
                action_manager.get_logger().info("Service call failed %r" % (e,))
            break

    client.loop_stop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
