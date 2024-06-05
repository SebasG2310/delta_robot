import time
import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
import threading

class DeltaRobotController(Node):
    def __init__(self):
        super().__init__('delta_robot_controller')
        self.homing_complete = False
        self.home_subscriber = self.create_subscription(
            Bool,
            'homing_complete',
            self.homing_callback,
            10)
        self.get_logger().info("Waiting for homing to complete.")
        self.coordinate_input_thread = threading.Thread(target=self.coordinate_input_loop)
        self.coordinate_input_thread.start()
        # Constants
        self.f =440 # Example value for f
        self.e = 100      # Example value for e
        self.rf = 230    # Example value for rf
        self.re = 400    # Example value for re
        self.pi = math.pi

        # Create a publisher to publish motor angles
        self.publisher = self.create_publisher(Float32MultiArray, 'motor_angles', 10)

        self.trajectory = [
            (0, 0, -300),  # Example coordinates for point 1
            (0, 0, -500),  # Example coordinates for point 2
            (0,0, -300)  # Example coordinates for point 3
        ]

    def homing_callback(self, msg):
        if msg.data == True:
            self.homing_complete = True
            self.get_logger().info("Homing complete. Updating home angles.")
            self.save_theta(-30, -30, -30)
    def coordinate_input_loop(self):
        while not self.homing_complete:
            time.sleep(0.5)
        while True:
            self.current_theta1, self.current_theta2, self.current_theta3 = self.read_theta()
            self.x_target, self.y_target, self.z_target = self.get_target_coordinates()
            if self.x_target is not None and self.y_target is not None and self.z_target is not None:
                self.calculate_and_publish_angles()
    def follow_trajectory(self):
        for point in self.trajectory:
            self.current_theta1, self.current_theta2, self.current_theta3 = self.read_theta()
            self.x_target, self.y_target, self.z_target = point
            self.calculate_and_publish_angles()
            time.sleep(3)  # Adjust sleep time as needed
    def get_target_coordinates(self):
        while True:
            try:
                x_target = float(input("Ingrese la coordenada X del objetivo: "))
                y_target = float(input("Ingrese la coordenada Y del objetivo: "))
                z_target = float(input("Ingrese la coordenada Z del objetivo: "))
                return x_target, y_target, z_target
            except ValueError:
                print("Por favor, ingrese coordenadas válidas.")
                return self.get_target_coordinates()

    def delta_calcAngleYZ(self, x0, y0, z0):
        y1 = -0.5 * 0.57735 * self.f  # f/2 * tg 30
        y0 -= 0.5 * 0.57735 * self.e   # shift center to edge
        a = (x0*x0 + y0*
        y0 + z0*z0 + self.rf*self.rf - self.re*self.re - y1*y1)/(2*z0)
        b = (y1 - y0)/z0
        d = -(a + b*y1)**2 + self.rf * (b*b*self.rf + self.rf)
        if d < 0:
            return -1, 0  # Non-existing point
        yj = (y1 - a*b - math.sqrt(d))/(b*b + 1)  # Choosing outer point
        zj = a + b*yj
        theta = 180.0 * math.atan(-zj/(y1 - yj))/self.pi + (180.0 if yj > y1 else 0.0)
        theta = max(theta,-30)
        return 0, theta

    def delta_calcInverse(self, x0, y0, z0):
        delta_theta1_desired, theta1_desired = self.delta_calcAngleYZ(x0, y0, z0)
        if delta_theta1_desired == -1:
            return -1, 0, 0, 0  # Non-existing position
        
        delta_theta2_desired, theta2_desired = self.delta_calcAngleYZ(x0*math.cos(2*self.pi/3) + y0*math.sin(2*self.pi/3), y0*math.cos(2*self.pi/3) - x0*math.sin(2*self.pi/3), z0)
        if delta_theta2_desired == -1:
            return -1, 0, 0, 0  # Non-existing position
        
        delta_theta3_desired, theta3_desired = self.delta_calcAngleYZ(x0*math.cos(2*self.pi/3) - y0*math.sin(2*self.pi/3), y0*math.cos(2*self.pi/3) + x0*math.sin(2*self.pi/3), z0)
        if delta_theta3_desired == -1:
            return -1, 0, 0, 0  # Non-existing position
        
        # Calculate the difference between current and desired angles
        delta_theta1 = theta1_desired - self.current_theta1
        delta_theta2 = theta2_desired - self.current_theta2
        delta_theta3 = theta3_desired - self.current_theta3
        
        return 0, delta_theta1, delta_theta2, delta_theta3

    def read_theta(self):
        try:
            with open("current_theta.txt", "r") as f:
                lines = f.readlines()
                theta1 = float(lines[0])
                theta2 = float(lines[1])
                theta3 = float(lines[2])
                return theta1, theta2, theta3
        except FileNotFoundError:
            print("Current theta file not found. Using default values.")
            return 0, 0, 0

    def save_theta(self, theta1, theta2, theta3):
        with open("current_theta.txt", "w") as f:
            f.write(f"{theta1}\n{theta2}\n{theta3}")

    def calculate_and_publish_angles(self):
        # Calculate joint angles for the target position
        status, theta1, theta2, theta3 = self.delta_calcInverse(self.x_target, self.y_target, self.z_target)

        if status == 0:
            # Display current angles, changes, and desired angles
            print('Current angles: ', self.current_theta1, self.current_theta2, self.current_theta3)
            print('Changes in angles: ', theta1, theta2, theta3)
            print('Desired angles: ', theta1 + self.current_theta1, theta2 + self.current_theta2,
                  theta3 + self.current_theta3)

            # Save new theta values to file
            self.save_theta(theta1 + self.current_theta1, theta2 + self.current_theta2, theta3 + self.current_theta3)

            # Publish calculated angles
            msg = Float32MultiArray()
            msg.data = [theta1, theta2, theta3]
            self.publisher.publish(msg)
        else:
            print("Non-existing position")

def main(args=None):
    rclpy.init(args=args)
    delta_robot_controller = DeltaRobotController()
    rclpy.spin(delta_robot_controller)
    delta_robot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
