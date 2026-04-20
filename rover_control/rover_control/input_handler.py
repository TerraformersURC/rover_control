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
            UI, '/controls', self.ui_input_callback, 10
        )
        self.joy_subscription = self.create_subscription(
            Joy, '/joy', self.joy_input_callback, 10
        )
        self.total_cameras = TOTAL_CAMERAS
        self.camera_index = 0
        self.prev_lb = 0
        self.prev_rb = 0
        self.joystick_mode = 0
        self.prev_back = 0

        self.default_speed = 0.5

    def joy_input_callback(self, msg: Joy):
        left_y  = msg.axes[1]
        right_y = msg.axes[3]
        rt      = msg.axes[5]
        lb      = msg.buttons[4]
        rb      = msg.buttons[5]

        a_button = msg.buttons[1]
        b_button = msg.buttons[2]
        x_button = msg.buttons[3]
        y_button = msg.buttons[4]

        back_button = msg.buttons[8]
        start_button = msg.buttons[9]

        out = Controller()

        # ------------------------------
        #           BUMPERS
        # ------------------------------

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

        out.camera_num = self.camera_index

        # ------------------------------
        #           TRIGGERS
        # ------------------------------

        out.end_effector  = rt

        # ------------------------------
        #          BACK BUTTON
        # ------------------------------

        if back_button == 1 and self.prev_back == 0:
            self.joystick_mode = not self.joystick_mode
        self.prev_back = back_button

        out.joystick_mode = self.joystick_mode

        # ------------------------------
        #           JOYSTICKS
        # ------------------------------

        if (self.joystick_mode == 0):
            out.drive_left = left_y
            out.drive_right = right_y


        
        self.publisher.publish(out)

    def ui_input_callback(self, msg: UI):
        out = Controller()

        if msg.drivetrain_fwd:
            out.drive_left  =  self.default_speed
            out.drive_right =  self.default_speed
        elif msg.drivetrain_rev:
            out.drive_left  = -self.default_speed
            out.drive_right = -self.default_speed
        elif msg.drivetrain_left:
            out.drive_left  =  0.0
            out.drive_right =  self.default_speed
        elif msg.drivetrain_right:
            out.drive_left  =  self.default_speed
            out.drive_right =  0.0

        self.publisher.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = ConvertInputs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()