import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from .settings import Settings
from interfaces.srv import StartRecording
from interfaces.srv import StopRecording

class SubmarineNode(Node):
    def __init__(self):
        super().__init__("submarine_node")
        self.settings = Settings()
        self.is_recording = False

        self.declare_parameter("speed", 10.0)
        self.settings.submarine_speed = self.get_parameter("speed").value

        self.pos_publisher = self.create_publisher(Pose2D, "/position", 10)
        self.start_record_client = self.create_client(StartRecording, "start_recording")
        self.stop_record_client = self.create_client(StopRecording, "stop_recording")

        self.listen_to_terminal()
        
    def listen_to_terminal(self):
        print("Enter command for submarine \n - W/S/A/D - Movement \n Q - Quit \n R - Record")
        while True:
            command = input()
            self.process_command(command)

    def process_command(self, command):
        self.get_logger().info(f"Command: {command}")
        pose = Pose2D()

        if(command == 'w'):
            pose.x = 0.0
            pose.y = float(-self.settings.submarine_speed)
        elif(command == 's'):
            pose.x = 0.0
            pose.y = float(self.settings.submarine_speed) 
        elif(command == 'a'):
            pose.x = float(-self.settings.submarine_speed) 
            pose.y = 0.0
        elif(command == 'd'):
            pose.x = float(self.settings.submarine_speed)
            pose.y = 0.0
        elif(command == 'q'):
            pose.x = -1.0
            pose.y = -1.0
        elif(command == 'r'):
            req = StartRecording.Request()
            req.start_recording = True
            self.start_record_client.call_async(req)
        elif(command == 'p'):
            req = StopRecording.Request()
            req.stop_recording = True
            self.stop_record_client.call_async(req)
        self.pos_publisher.publish(pose)


def main(args=None):
    rclpy.init(args=args)

    node = SubmarineNode()
    rclpy.spin(node)

    rclpy.shutdown()