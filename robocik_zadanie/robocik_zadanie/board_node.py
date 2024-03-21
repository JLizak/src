import sys
import pygame
import rclpy
from rclpy.node import Node
from .settings import Settings
from .submarine import Submarine
from geometry_msgs.msg import Pose2D
from interfaces.srv import StartRecording, StopRecording

class BoardNode(Node):
    def __init__(self):
        super().__init__('board_node')
        pygame.init()
        self.settings = Settings()
        
        # left border, right border, top border, bottom border
        self.declare_parameter("forbidden_areas", [0, self.settings.screen_width - 70, -30, self.settings.screen_height-60])
        self.declare_parameter("lives", 5)

        self.forbidden_areas = self.get_parameter("forbidden_areas").value
        self.lives = self.get_parameter("lives").value

        self.get_logger().info("Board Node has been started")
        self.pos_subscriber = self.create_subscription(Pose2D, 'position', self.pos_callback, 10)
        
        self.screen = pygame.display.set_mode((self.settings.screen_width, self.settings.screen_height))
        pygame.display.set_caption("Submarines")
        self.submarine = Submarine(game=self)

        self.is_recording = False
        self.recorded_pos = []
        self.record_service = self.create_service(StartRecording, "start_recording", self.start_recording_callback)
        self.stop_record_service = self.create_service(StopRecording, "stop_recording", self.stop_recording_callback)

    def run_event_loop(self):
        while True:
            rclpy.spin_once(self, timeout_sec=0.0)
            for event in pygame.event.get():
                if event.type == pygame.QUIT:   
                    pygame.quit()
                    rclpy.shutdown()
                    sys.exit()
        
            self._update_screen()

    def pos_callback(self, pose: Pose2D):
        if pose.x == -1.0 and pose.y == -1.0:
            self.get_logger().info("Recorded Positions:")
            for pos in self.recorded_pos:
                msg = f"{pos[0]}, {pos[1]}"
                self.get_logger().info(msg)
            pygame.quit()
            rclpy.shutdown()
            sys.exit()

        else:
            new_x = self.submarine.rect.x + int(pose.x)
            new_y = self.submarine.rect.y + int(pose.y)

            if(self.check_borders(new_x, new_y)):
                self.submarine.rect.x = new_x
                self.submarine.rect.y = new_y
            else:
                self.lives -= 1
                self.get_logger().warn(f"Out of borders! Lives remaining: {self.lives}")        
                if self.lives <= 0:
                    self.get_logger.error("No more lives!")
                    pygame.quit()
                    rclpy.shutdown()
                    sys.exit()

            if(self.is_recording): 
                self.recorded_pos.append((self.submarine.rect.x, self.submarine.rect.y))
    
    def check_borders(self, x, y):
        left, right, top, bottom = self.forbidden_areas
        if left < x < right and top < y < bottom:
            return True
        return False


    def start_recording_callback(self, request, response):
        self.is_recording = True
        response.message = "Recording started"
        return response

    def stop_recording_callback(self, request, response):
        self.is_recording = False
        response.message = "Recording stopped"
        return response

    def _update_screen(self):
        self.screen.fill(self.settings.bg_color)
        self.submarine.blitme()
        pygame.display.flip()

def main(args=None):
    rclpy.init(args=args)
    node = BoardNode()
    node.run_event_loop()

    rclpy.shutdown()
if __name__ == "__main__":
    main()
