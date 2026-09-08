import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from rover_interface.msg import Controller, UI
from rover_control.controller_bindings import BINDINGS

TOTAL_CONTROL_MODES = 3     # drivetrain & camera, arm, science
TOTAL_CAMERAS = 2           # The number of cameras that we want to be able to switch between
PUBLISH_RATE = 20.0         # Controller messages published per second
CONTROLLER = 2              # Which controller profile to use (1 or 2, see controller_bindings.py)
ARM_XYZ_SCALE = 0.03
ARM_RPY_SCALE = 0.1

# End effector (arm mode, sent to the Arduino via arduino_writer)
EE_RATE = 20.0              # end effector degrees per second at full trigger.
                            # What goes on the wire is a per-message delta, so at
                            # PUBLISH_RATE this is 1 degree per message: the same
                            # step size the d-pad sends for the camera servos.
                            # arduino_writer owns the resulting position and its
                            # limits, so there is nothing to clamp here.
TRIGGER_DEADZONE = 0.05     # fraction of trigger travel ignored, stops creep

# ------------------------------
#          DRIVETRAIN FEEL
# ------------------------------
# Everything about how the sticks map to motor output is tuned here. The
# drivetrain bridge just clamps and forwards whatever it is sent, so these are
# the only numbers that need touching.
DRIVE_MAX_OUTPUT = 0.40     # top speed as a fraction of full throttle
DRIVE_DEADZONE = 0.10       # stick travel around centre that counts as zero
DRIVE_EXPO = 0.6            # 0.0 = linear, 1.0 = fully cubic (fine control near centre)
DRIVE_SLEW_RATE = 0.75      # max change in output per second, in full-throttle units.
                            # At 0.75 a full stop-to-DRIVE_MAX_OUTPUT change takes
                            # DRIVE_MAX_OUTPUT / DRIVE_SLEW_RATE seconds.

# Science mode (Arduino subsystems, see jetson_arduino_comm)
DRILL_MAX_SPEED = 255       # magnitude sent to the Arduino, 0-255
SCREW_MAX_SPEED = 255
SCIENCE_DEADZONE = 0.15     # sticks rest near zero but rarely exactly zero
STEPS_PER_REV = 200         # must match STEPS_PER_REV in trfm_arduino_26_combined.ino
STEPS_PER_PRESS = 1         # turntable steps advanced per d-pad press.
                            # This is the knob for turntable feel: 1 step is
                            # 1.8 degrees, 50 would reproduce the old 90 degree
                            # cells. Sender-side only, no reflash to change.

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

        # Science mode edge detection (separate from the camera-switch lb/rb
        # state so switching modes doesn't fire a spurious edge)
        self.prev_dpad_x = 0.0

        # Drivetrain: where the sticks are asking to go, and where the ramped
        # output actually is. The gap between them is closed in the timer.
        self.drive_target_left = 0.0
        self.drive_target_right = 0.0
        self.drive_left = 0.0
        self.drive_right = 0.0

        # Latched Arduino state, held across all control modes
        self.turntable_step = 0
        self.ee_velocity = 0          # degrees per second, + opens, - closes,
                                        # set in arm mode
        self.trigger_armed = {'lt': False, 'rt': False}

        self.default_speed = 0.5

        self.out = Controller()

    def timer_callback(self):
        self.ramp_drive()
        self.apply_arduino_state(self.out)
        self.publisher.publish(self.out)

    @staticmethod
    def shape_drive_axis(value: float) -> float:
        """Stick axis (-1.0 to 1.0) to a motor output.

        Three things happen here: the deadzone is subtracted and the remaining
        travel rescaled so the rover doesn't creep on a stick that rests off
        centre, an expo curve flattens the response near centre so small
        corrections stay small, and the result is scaled to DRIVE_MAX_OUTPUT.
        """
        if abs(value) < DRIVE_DEADZONE:
            return 0.0
        scaled = (abs(value) - DRIVE_DEADZONE) / (1.0 - DRIVE_DEADZONE)
        scaled = min(scaled, 1.0)
        curved = (1.0 - DRIVE_EXPO) * scaled + DRIVE_EXPO * scaled ** 3
        output = curved * DRIVE_MAX_OUTPUT
        return -output if value < 0 else output

    def ramp_drive(self):
        """Slew the drive output toward the stick target at the publish rate.

        Like the end effector, this can't live in the joy callback: holding a
        stick steady produces no further joy messages, so the ramp would stall
        partway. Limiting the rate here rather than letting the sticks step
        straight through is what stops the jerk on direction changes.
        """
        if self.control_mode == 0:
            target_left = self.drive_target_left
            target_right = self.drive_target_right
        else:
            # Not in drivetrain mode: coast to a stop rather than cutting out.
            target_left = 0.0
            target_right = 0.0

        step = DRIVE_SLEW_RATE / PUBLISH_RATE
        self.drive_left = self.slew(self.drive_left, target_left, step)
        self.drive_right = self.slew(self.drive_right, target_right, step)

        self.out.drive_left = self.drive_left
        self.out.drive_right = self.drive_right

    @staticmethod
    def slew(current: float, target: float, max_step: float) -> float:
        """Move current toward target by at most max_step."""
        delta = target - current
        if delta > max_step:
            return current + max_step
        if delta < -max_step:
            return current - max_step
        return target

    def trigger_pressed(self, msg: Joy, name: str) -> float:
        """Trigger axis to a 0.0-1.0 pressed fraction.

        A trigger reads 1.0 at rest and -1.0 fully pressed, but an untouched
        one reads exactly 0.0 until first pressed, which is the midpoint of
        that travel. The first nonzero reading arms it; until then it reports
        unpressed so nothing moves on its own.
        """
        value = msg.axes[self.axes[name]]
        if value != 0.0:
            self.trigger_armed[name] = True
        if not self.trigger_armed[name]:
            return 0.0

        pressed = min(max((1.0 - value) / 2.0, 0.0), 1.0)
        return pressed if pressed >= TRIGGER_DEADZONE else 0.0

    def joy_input_drivetrain(self, msg: Joy) -> Controller:
        out: Controller = Controller()
        out.control_mode = 0

        # ------------------------------
        #          DRIVETRAIN
        # ------------------------------
        # Only the target is set here. The published value is ramped toward it
        # in ramp_drive() so the output can keep moving between joy messages.
        self.drive_target_left = self.shape_drive_axis(msg.axes[self.axes['left_y']])
        self.drive_target_right = self.shape_drive_axis(msg.axes[self.axes['right_y']])

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

        # camera_num is written in apply_arduino_state, so it survives a mode
        # switch instead of resetting to 0.

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

        # ------------------------------
        #          END EFFECTOR
        # ------------------------------
        # Right trigger opens, hold rb to close instead, speed proportional to
        # how far the trigger is pressed. Mirrors the lt/lb convention used for
        # arm z above; lt itself is taken, so the gripper lives on the right
        # side. Deliberately outside the arm_mode check: the gripper is its own
        # servo and should work whether the arm is being posed or jogged.
        # Release the trigger and the gripper holds position, since zero
        # velocity is zero delta and arduino_writer keeps the position it has
        # already accumulated. Converted to a per-message delta in
        # apply_arduino_state.
        pressed = self.trigger_pressed(msg, 'rt')
        if msg.buttons[self.buttons['rb']] == 1:
            pressed = -pressed
        if pressed > 0.0:
            self.ee_velocity = 1
        elif pressed < 0.0:
            self.ee_velocity = -1
        else:
            self.ee_velocity = 0

        return out

    @staticmethod
    def axis_to_speed(value: float, max_speed: int) -> int:
        """Stick axis (-1.0 to 1.0) to a signed Arduino speed.

        The deadzone is subtracted and the remaining travel rescaled, so the
        motor doesn't jump straight to 15% the moment the stick leaves centre.
        """
        if abs(value) < SCIENCE_DEADZONE:
            return 0
        scaled = (abs(value) - SCIENCE_DEADZONE) / (1.0 - SCIENCE_DEADZONE)
        speed = int(round(scaled * max_speed))
        return -speed if value < 0 else speed

    def joy_input_science(self, msg: Joy) -> Controller:
        out: Controller = Controller()
        out.control_mode = 2

        # ------------------------------
        #          AUGER DRILL
        # ------------------------------
        # Sticks, not triggers: an untouched trigger axis reads 0.0 until it is
        # first pressed, which would spin the drill up on its own.
        out.drill_speed = self.axis_to_speed(
            msg.axes[self.axes['right_y']], DRILL_MAX_SPEED
        )

        # ------------------------------
        #          LEAD SCREW
        # ------------------------------
        out.screw_speed = self.axis_to_speed(
            msg.axes[self.axes['left_y']], SCREW_MAX_SPEED
        )

        # ------------------------------
        #          TURNTABLE
        # ------------------------------
        # STEPS_PER_PRESS per d-pad press, wrapping both ways. What goes on the
        # wire is the resulting absolute step position, not the increment, so a
        # dropped frame just gets re-applied. The Arduino takes the shortest
        # path to whatever position it is given.
        dpad_x = msg.axes[self.axes['dpad_x']]
        if dpad_x > 0.5 and self.prev_dpad_x <= 0.5:
            self.turntable_step = (self.turntable_step + STEPS_PER_PRESS) % STEPS_PER_REV
        elif dpad_x < -0.5 and self.prev_dpad_x >= -0.5:
            self.turntable_step = (self.turntable_step - STEPS_PER_PRESS) % STEPS_PER_REV
        self.prev_dpad_x = dpad_x

        return out

    def apply_arduino_state(self, out: Controller):
        """Resend the absolute/latched state on every message.

        Each mode builds a fresh Controller(), so anything not rewritten here
        falls back to the message default and is read downstream as a real
        command. The turntable is an absolute-position command, so the default
        would drive it back to cell 0 the moment you switch out of science
        mode. camera_num is the same problem one step removed: camera_manager
        reads it on every message, so the default would flip the feed back to
        camera 0 whenever you left drivetrain mode.

        end_effector_delta is the opposite case: arduino_writer holds the
        position and adds whatever delta each message carries, so the default
        of 0.0 is exactly right for every mode that isn't driving the gripper.
        It has to be written here rather than in the joy callback because
        holding a trigger steady produces no further joy messages, and a
        gripper that only moved on new joy frames would stop after one tick.
        """
        out.gripper_delta = self.ee_velocity

    def joy_input_callback(self, msg: Joy):
        # update control mode
        start = msg.buttons[self.buttons['start']]
        if start == 1 and self.prev_start == 0:
            self.control_mode += 1
            if self.control_mode >= TOTAL_CONTROL_MODES:
                self.control_mode = 0
        self.prev_start = start

        # Any mode that isn't arm leaves the gripper stationary. Without this,
        # switching modes while holding the trigger would leave the timer
        # publishing a nonzero delta with nothing left to update it.
        self.ee_velocity = 0

        # generate a different controller message depending on control mode
        if self.control_mode == 0:
            self.out = self.joy_input_drivetrain(msg)
        elif self.control_mode == 1:
            self.out = self.joy_input_arm(msg)
        elif self.control_mode == 2:
            self.out = self.joy_input_science(msg)

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