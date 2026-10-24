# Rover Control

## Dependencies

- Clone the `rover_interface` repository into your workspace
- Install ROS joy node for jazzy: `sudo apt install ros-jazzy-joy`

## Launching

- Launch the ROS joy node: `ros2 run joy joy_node`
- Launch the input handler: `ros2 run rover_control input_handler`

The joy node is necessary to turn controller inputs into a ROS topic. The input handler subscribes to this topic and publishes the controller topic.

## Controls

- **Start**: Switch through control modes
- **Mode**: Swap functionality of Left Stick and D-pad

### Control Mode 0 - Drivetrain & Camera

- **Left Stick**: Back/forward for the left side of the rover
- **Right Stick**: Back/forward for the right side of the rover
- **RB/LB**: Next/previous camera
- **D-pad**: Aim camera

### Control Mode 1 - Arm

- **D-pad Up/Down**: Arm enable / disable
- **D-pad Right**: Preset pose mode
- **D-pad Left**: Velocity jog mode
- **RT**: Open gripper
- **RT + RB**: Close gripper
- **Preset Pose Mode**: A = pose 0, X = pose 1, B = pose 2
- **Velocity Jog Mode**:
- **Left Stick X**: Arm x
- **Left Stick Y**: Arm y
- **LT**: Arm z (one direction)
- **LT + LB**: Arm z (opposite direction)

### Control Mode 2 - Drill

- **Right Stick Y**: Auger drill speed
- **Left Stick Y**: Lead screw speed
- **D-pad Right/Left**: Turntable step forward/back

## Important Notes

- Put important notes here!
