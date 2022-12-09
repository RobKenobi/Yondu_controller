import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo

from tello_msgs.srv import TelloAction
from rclpy.qos import qos_profile_sensor_data
# Package to convert between ROS and OpenCV Images
from cv_bridge import CvBridge, CvBridgeError
import cv2  # OpenCV library
import cv2.aruco as aruco

from Pid import PID
import numpy as np
import matplotlib.pyplot as plt

save = [[],[],[]]


# Aruco
a = 0.17  # m
aruco_marker_points = np.array([[-a / 2, a / 2, 0],
                                [a / 2, a / 2, 0],
                                [a / 2, -a / 2, 0],
                                [-a / 2, -a / 2, 0]])


def get_position(position):
    x = position[0, 0]
    y = position[1, 0]
    z = position[2, 0]
    return x, y, z

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


        # Camera intrinsic parameters
        self.distCoeffs = np.array([-0.068230, 0.075141, -0.005676, 0.003996, 0.0])

        self.mtx = np.array([[879.064319, 0.0, 500.597355], 
                             [0.0, 876.381281, 367.439947], 
                             [0.0, 0.0, 1.0]])

        # PID definition
        Ku = 1.5
        self.pid_vx = PID(Ku/2, Ku/15, 0.0)
        self.pid_vy = PID(Ku/2, Ku/25, 0.0)
        self.pid_vz = PID(1.1, 0.35, 0.0)
       
        # Last tag seen
        self.last_tag_seen = None
        self.change_tag = False
        self.last_time = None
        self.time_on_target = None
        self.reach_target = False

        # CREATE THE SUBSCRIBER FOR IMAGE RECEPTION
        qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                          history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                          depth=1)
        self.image_subscription_ = self.create_subscription(
            Image, '/drone1/image_raw', self.image_callback, qos_profile=qos_policy)
        self.display_image = True
        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def cmd_vel_loop(self):
        """
        I'm used to publish every 0.5 seconds the velocity command.
        You don't need to modify me, just modify vx, vy and v_yaw when you need it.
        """
        time_security = 5.0
        try:
            corners = self.corners
            gray = self.gray
            ids = self.ids

            if len(corners) > 0:
                print("TAG SEEN %s" % ids)

                if self.change_tag is True:
                    if ids[0] != self.last_tag_seen:
                        self.change_tag = False
                    else:
                        speed = 1.0
                        # Waiting 5 seconds to reach the tag
                        if time.time() - self.last_time < time_security :
                            # TAG 3 --> going up
                            if self.last_tag_seen == 3 :
                                print("Looking for tag 2")
                                self.vx = 0.0
                                self.vy = 0.0
                                self.vz = speed
                            # TAG 2 --> going left
                            elif self.last_tag_seen == 2:
                                print("Looking for tag 1")
                                self.vx = 0.0
                                self.vy = speed
                                self.vz = 0.0
                            # TAG 1 --> going down
                            elif self.last_tag_seen == 1 :
                                print("Looking for tag 4")
                                self.vx = 0.0
                                self.vy = 0.0
                                self.vz = -speed
                            # TAG 4 --> going right
                            else:
                                print("Looking for tag 3")
                                self.vx = 0.0
                                self.vy = -speed
                                self.vz = 0.0
                        # else stop the drone
                        else:
                            print("STOP")
                            self.vx = 0.0
                            self.vy = 0.0
                            self.vz = 0.0

                if self.change_tag is False:
                    # TODO : Here, write your controller
                    # From the corners information, set the right command speed
                    # Like self.vx, self.vy, self.v_yaw, to control the UAV

                    # rvec and tvec are rotation and translation vectors
                    # from a 3D point in the world frame to the camera frame
                    # In our case, it is the translation matrix from the aruco marker frame
                    # to the camera frame
                    _, rvec, tvec = cv2.solvePnP(objectPoints=aruco_marker_points,
                                                imagePoints=corners[0],
                                                cameraMatrix=self.mtx,
                                                distCoeffs=self.distCoeffs)
                    x,y,z = get_position(tvec)
                    
                    error_x = z - 2
                    error_y = - x
                    error_z = - y
                    print(f"Error x : {round(error_x,3)} Error y : {round(error_y,3)} Error z : {round(error_z,3)}")

                    threshold = 1e-2
                    if abs(error_x)<threshold and abs(error_y<threshold) and (error_z<threshold):
                        if self.reach_target is False:
                            self.reach_target = True
                            self.time_on_target = time.time()
                        else :
                            if time.time() - self.time_on_target > 3:
                                self.reach_target = False
                                self.change_tag = True
                                self.last_tag_seen = ids[0]
                                self.last_time = time.time()
                    
                    save[0].append(error_x)
                    save[1].append(error_y)
                    save[2].append(error_z)
                    self.vx = self.pid_vx.loop(error_x)
                    self.vy = self.pid_vy.loop(error_y)
                    self.vz = self.pid_vz.loop(error_z)

            else:
                print("No tag seen..")
                # The UAV stay in the same pose, it doesn't move
                # Bas droite TAG 3
                # Haut droite TAG 2
                # Haut gauche TAG 1
                # Bas gauche TAG 4

                speed = 1.0
                # Waiting 5 seconds to reach the tag
                if time.time() - self.last_time < time_security :
                    # TAG 3 --> going up
                    if self.last_tag_seen == 3 :
                        print("Looking for tag 2")
                        self.vx = 0.0
                        self.vy = 0.0
                        self.vz = speed
                    # TAG 2 --> going left
                    elif self.last_tag_seen == 2:
                        print("Looking for tag 1")
                        self.vx = 0.0
                        self.vy = speed
                        self.vz = 0.0
                    # TAG 1 --> going down
                    elif self.last_tag_seen == 1 :
                        print("Looking for tag 4")
                        self.vx = 0.0
                        self.vy = 0.0
                        self.vz = -speed
                    # TAG 4 --> going right
                    else:
                        print("Looking for tag 3")
                        self.vx = 0.0
                        self.vy = -speed
                        self.vz = 0.0
                # else stop the drone
                else:
                    print("STOP")
                    self.vx = 0.0
                    self.vy = 0.0
                    self.vz = 0.0
        except:
            pass

        msg = Twist()
        msg.linear.x = self.vx
        msg.linear.y = self.vy
        msg.linear.z = self.vz
        msg.angular.z = self.v_yaw
        self.cmd_vel_publisher_.publish(msg)

    def image_callback(self, data):
        """
        I'm used to get camera's image.
        I can set the desired velocity according to the aruco detection
        """
        try:
            cv_image = self.br.imgmsg_to_cv2(data)
            # Our operations on the frame come here
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_1000)
            parameters = aruco.DetectorParameters_create()

            # lists of ids and the corners beloning to each id
            corners, ids, rejectedImgPoints = aruco.detectMarkers(
                gray, aruco_dict, parameters=parameters)

            gray = aruco.drawDetectedMarkers(gray, corners)
            

            self.corners = corners
            self.gray = gray
            self.ids = ids

            # Display the resulting frame
            if self.display_image:
                cv2.imshow('frame', self.gray)
                cv2.waitKey(3)
            
        except CvBridgeError as e:
            pass


def main(args=None):
    rclpy.init(args=args)

    # Wait for take off service to be ready
    action_manager = ActionManager()
    action_manager.ask_for_takeoff()
    ready_to_continue_mission = False

    # Try to takeoff, wait for the return of the service
    while rclpy.ok():
        rclpy.spin_once(action_manager)
        if action_manager.future.done():
            try:
                response = action_manager.future.result()
                if response.rc == 1:  # OK
                    print("Take off is a success !")
                    ready_to_continue_mission = True
                else:  # NOT OK
                    print("Something is wrong with the takeoff. LANDING NOW")
            except Exception as e:
                action_manager.get_logger().info("Service call failed %r" % (e,))
            break

    if ready_to_continue_mission:
        controller = Controller()
        try:
            while rclpy.ok():
                rclpy.spin_once(controller)
        except KeyboardInterrupt:
            np.save("DATA.npy", np.array(save))
            print("Stopping the control. Ask for landing.")
        controller.destroy_node()

    # ASK FOR LANDING
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

    rclpy.shutdown()


if __name__ == '__main__':
    main()
