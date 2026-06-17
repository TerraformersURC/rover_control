import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rover_interface.msg import Controller, UI
from rover_control.controller_bindings import BINDINGS

TOTAL_CONTROL_MODES = 2     # drivetrain & camera, arm
TOTAL_CAMERAS = 2           # The number of cameras that we want to be able to switch between
PUBLISH_RATE = 20.0         # Controller messages published per second
CONTROLLER = 2              # Which controller profile to use (1 or 2, see controller_bindings.py)
ARM_XYZ_SCALE = 0.05   
ARM_RPY_SCALE = 0.1

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

        self.timer = self.create_timer(1.0 / PUBLISH_RATE, self.timer_callback)

        self.axes = BINDINGS[CONTROLLER]['axes']
        self.buttons = BINDINGS[CONTROLLER]['buttons']

        self.total_cameras = TOTAL_CAMERAS
        self.camera_index = 0
        self.prev_lb = 0
        self.prev_rb = 0
        self.control_mode = 0
        self.prev_start = 0
        self.arm_enabled = False
        self.arm_mode = 0

        self.default_speed = 0.5

        self.out = Controller()

    def timer_callback(self):
        self.publisher.publish(self.out)

    def joy_input_drivetrain(self, msg: Joy) -> Controller:
        out: Controller = Controller()
        out.control_mode = 0

        # ------------------------------
        #          DRIVETRAIN
        # ------------------------------
        out.drive_left = msg.axes[self.axes['left_y']]
        out.drive_right = msg.axes[self.axes['right_y']]

        # ------------------------------
        #          CAMERA SWITCHING
        # ------------------------------
        lb = msg.buttons[self.buttons['lb']]
        rb = msg.buttons[self.buttons['rb']]
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
        #          CAMERA MOVEMENT
        # ------------------------------
        dpad_x = msg.axes[self.axes['dpad_x']]
        dpad_y = msg.axes[self.axes['dpad_y']]
        out.camera_left_right = dpad_x
        out.camera_up_down = dpad_y


        return out

    def joy_input_arm(self, msg: Joy) -> Controller:
        out: Controller = Controller()
        out.control_mode = 1

        # ------------------------------
        #          ARM ENABLE
        # ------------------------------
        dpad_y = msg.axes[self.axes['dpad_y']]
        # used < 0.5 because of floating point in case dpad ever evals to -.999 or 0.0001 etc
        if dpad_y < -0.5:
            self.arm_enabled = False
        elif dpad_y > 0.5:
            self.arm_enabled = True
        out.arm_enabled = self.arm_enabled

        # ------------------------------
        #          ARM CONTROL MODE
        # ------------------------------
        dpad_x = msg.axes[self.axes['dpad_x']]
        if dpad_x > 0.5:
            self.arm_mode = 0
        elif dpad_x < -0.5:
            self.arm_mode = 2
        out.arm_mode = self.arm_mode

        # ------------------------------
        #         ARM VELOCITY CONTROL
        # ------------------------------
        if self.arm_mode == 2:
            out.arm_goal_vel.linear.x = -msg.axes[self.axes['left_x']] * ARM_XYZ_SCALE
            out.arm_goal_vel.linear.y = msg.axes[self.axes['left_y']] * ARM_XYZ_SCALE

            vel_z = msg.axes[self.axes['lt']]
            vel_z -= 1.0
            vel_z /= 2.0
            vel_z *= ARM_XYZ_SCALE
            if msg.buttons[self.buttons['lb']] == 1:
                vel_z = -vel_z

            out.arm_goal_vel.linear.z = vel_z

        # ------------------------------
        #         ARM ID CONTROL
        # ------------------------------
        out.arm_target_id = -1 # no id by default
        if self.arm_mode == 0:
            if msg.buttons[self.buttons['a']]:
                out.arm_target_id = 0
            elif msg.buttons[self.buttons['x']]:
                out.arm_target_id = 1
            elif msg.buttons[self.buttons['b']]:
                out.arm_target_id = 2

        return out

    def joy_input_callback(self, msg: Joy):
        # update control mode
        start = msg.buttons[self.buttons['start']]
        if start == 1 and self.prev_start == 0:
            self.control_mode += 1
            if self.control_mode >= TOTAL_CONTROL_MODES:
                self.control_mode = 0
        self.prev_start = start

        # generate a different controller message depending on control mode
        if self.control_mode == 0:
            self.out = self.joy_input_drivetrain(msg)
        elif self.control_mode == 1:
            self.out = self.joy_input_arm(msg)

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

        # TEMPORARY: Buttons on the UI meant to controll the arm will instead be for the camera mount

        if msg.arm_left:
            out.camera_left_right  =  -1.0
        elif msg.arm_right:
            out.camera_left_right  =  1.0
        elif msg.arm_up:
            out.camera_up_down  =  1.0
        elif msg.arm_down:
            out.camera_up_down  =  -1.0

def main(args=None):
    rclpy.init(args=args)
    node = ConvertInputs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()