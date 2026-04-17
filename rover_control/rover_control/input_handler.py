import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rover_interface.msg import Controller, UI

TOTAL_CAMERAS = 2           # The number of cameras that we want to be able to switch between

class ConvertInputs(Node):
    def __init__(self):
        super().__init__('convert_inputs')
        self.publisher = self.create_publisher(Controller, 'controller_topic', 10)
        self.ui_subscription = self.create_subscription(
            UI, '/controls', self.ui_callback, 10
        )
        self.joy_subscription = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10
        )
        self.total_cameras = TOTAL_CAMERAS
        self.camera_index = 0
        self.prev_lb = 0
        self.prev_rb = 0

    def joy_callback(self, msg: Joy):
        left_y  = msg.axes[1]
        right_y = msg.axes[3]
        rt      = msg.axes[5]
        lb      = msg.buttons[4]
        rb      = msg.buttons[5]

        if rb == 1 and self.prev_rb == 0:
            self.camera_index += 1
        if lb == 1 and self.prev_lb == 0:
            self.camera_index -= 1
        self.prev_rb = rb
        self.prev_lb = lb

        if (self.camera_index >= self.total_cameras):
            self.camera_index = 0
        elif (self.camera_index < 0):
            self.camera_index = self.total_cameras - 1

        out = Controller()
        out.joystick_mode = True
        out.drive_left    = left_y
        out.drive_right   = right_y
        out.end_effector  = rt
        out.camera_num    = self.camera_index
        self.publisher.publish(out)

    def ui_callback(self, msg: UI):
        default_speed = 0.5
        out = Controller()
        out.joystick_mode = False

        if msg.drivetrain_fwd:
            out.drive_left  =  default_speed
            out.drive_right =  default_speed
        elif msg.drivetrain_rev:
            out.drive_left  = -default_speed  # fix: was positive before
            out.drive_right = -default_speed
        elif msg.drivetrain_left:
            out.drive_left  =  0.0
            out.drive_right =  default_speed
        elif msg.drivetrain_right:
            out.drive_left  =  default_speed
            out.drive_right =  0.0

        self.publisher.publish(out)  # fix: was missing

def main(args=None):
    rclpy.init(args=args)
    node = ConvertInputs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()