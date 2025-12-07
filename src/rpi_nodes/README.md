# RPI Nodes Package

This ROS 2 package provides nodes for the Raspberry Pi to control vacuum robot motors and handle keyboard input.

## Nodes

### 1. Motor Direction Controller (`rpi_motor_dir`)
Controls 4 motors' direction using TB6612FNG motor drivers via Raspberry Pi GPIO pins. The node subscribes to cmd_vel messages and sets appropriate direction control pins.

### 2. Keyboard Control Node (`keyboard_control`)
Reads keyboard input and publishes integer values to 4 different topics based on key groups.

## Overview - Motor Direction Controller

The `rpi_motor_dir` node is designed to work in conjunction with the ESP32 vacuum robot firmware. While the ESP32 handles PWM speed control, this Raspberry Pi node handles direction control for 4 motors using TB6612FNG drivers.

## Hardware Setup

### TB6612FNG Motor Driver Connections

Each TB6612FNG can control 2 motors, so you'll need 2 TB6612FNG boards for 4 motors.

**GPIO Pin Assignments:**
- Motor 1 (Front Left): GPIO 5 (INA), GPIO 6 (INB)
- Motor 2 (Front Right): GPIO 13 (INA), GPIO 19 (INB)  
- Motor 3 (Rear Left): GPIO 26 (INA), GPIO 21 (INB)
- Motor 4 (Rear Right): GPIO 20 (INA), GPIO 16 (INB)

**TB6612FNG Truth Table:**
- INA=1, INB=0: Forward (Clockwise)
- INA=0, INB=1: Reverse (Counter-clockwise)
- INA=0, INB=0: Brake/Stop
- INA=1, INB=1: Brake/Stop

### Wiring Diagram

```
Raspberry Pi          TB6612FNG Board 1        TB6612FNG Board 2
GPIO 5      ────────  AIN1 (Motor 1)
GPIO 6      ────────  AIN2 (Motor 1)         
GPIO 13     ────────  BIN1 (Motor 2)
GPIO 19     ────────  BIN2 (Motor 2)
                                               AIN1 (Motor 3)  ──────── GPIO 26
                                               AIN2 (Motor 3)  ──────── GPIO 21  
                                               BIN1 (Motor 4)  ──────── GPIO 20
                                               BIN2 (Motor 4)  ──────── GPIO 16

3.3V        ────────  VCC                      VCC
GND         ────────  GND                      GND
12V         ────────  VM                       VM
```

## Dependencies

- ROS 2 (tested on Humble/Iron)
- `rclpy`
- `geometry_msgs`
- `RPi.GPIO`

## Installation

1. Clone this package into your ROS 2 workspace:
```bash
cd ~/ws_vaccum/src
# Package is already in the workspace
```

2. Install Python dependencies:
```bash
pip3 install RPi.GPIO
```

3. Build the workspace:
```bash
cd ~/ws_vaccum
colcon build --packages-select rpi_nodes
source install/setup.bash
```

## Usage

### Running the Node

1. **Direct execution:**
```bash
ros2 run rpi_nodes rpi_motor_dir
```

2. **Using launch file:**
```bash
ros2 launch rpi_nodes motor_direction_controller.launch.py
```

3. **With debug logging:**
```bash
ros2 launch rpi_nodes motor_direction_controller.launch.py log_level:=debug
```

### Auto-start on Boot (Systemd Service)

1. **Copy service file:**
```bash
sudo cp src/rpi_nodes/config/motor-direction-controller.service /etc/systemd/system/
```

2. **Edit service file (adjust paths if needed):**
```bash
sudo nano /etc/systemd/system/motor-direction-controller.service
```

3. **Enable and start service:**
```bash
sudo systemctl daemon-reload
sudo systemctl enable motor-direction-controller.service
sudo systemctl start motor-direction-controller.service
```

4. **Check service status:**
```bash
sudo systemctl status motor-direction-controller.service
```

5. **View logs:**
```bash
sudo journalctl -u motor-direction-controller.service -f
```

### Hardware Testing

Before running the ROS node, test the GPIO connections:

```bash
cd ~/ws_vaccum/src/rpi_nodes/scripts
sudo python3 test_gpio.py
```

This interactive script allows you to:
- Test individual motor direction controls
- Test movement patterns (forward, backward, turns, strafing)
- Blink all GPIO pins to verify connections

**Note:** The test script requires `sudo` for GPIO access.

### Testing

1. **Check if node is running:**
```bash
ros2 node list | grep motor_direction_controller
```

2. **Monitor topics:**
```bash
ros2 topic echo /vaccum_base_controller/cmd_vel_out
```

3. **Send test commands:**
```bash
# Forward motion
ros2 topic pub /vaccum_base_controller/cmd_vel_out geometry_msgs/msg/TwistStamped "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
twist:
  linear: {x: 0.5, y: 0.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: 0.0}"

# Rotation
ros2 topic pub /vaccum_base_controller/cmd_vel_out geometry_msgs/msg/TwistStamped "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
twist:
  linear: {x: 0.0, y: 0.0, z: 0.0}  
  angular: {x: 0.0, y: 0.0, z: 0.5}"

# Stop
ros2 topic pub /vaccum_base_controller/cmd_vel_out geometry_msgs/msg/TwistStamped "
header:
  stamp: {sec: 0, nanosec: 0}
  frame_id: ''
twist:
  linear: {x: 0.0, y: 0.0, z: 0.0}
  angular: {x: 0.0, y: 0.0, z: 0.0}"
```

## Configuration

### Robot Parameters (in the node code)

- `WHEEL_SEPARATION`: Distance between left and right wheels (default: 0.5m)
- `WHEEL_BASE`: Distance between front and rear wheels (default: 0.4m)  
- `VELOCITY_DEADZONE`: Minimum velocity to consider motor active (default: 0.01 m/s)

### GPIO Pin Configuration

You can modify the GPIO pin assignments in the `__init__` method of the `MotorDirectionController` class:

```python
# Motor 1 (Front Left)
self.MOTOR1_INA = 5   # Change as needed
self.MOTOR1_INB = 6   # Change as needed
# ... etc
```

## Integration with ESP32 Firmware

This node is designed to work with the ESP32 vacuum robot firmware:

1. **ESP32 responsibilities:**
   - Subscribe to `/vaccum_base_controller/cmd_vel_out`
   - Control motor PWM speed (4 channels)
   - Handle encoders and feedback

2. **Raspberry Pi responsibilities:**
   - Control motor direction pins (8 GPIO pins)
   - Subscribe to same `/vaccum_base_controller/cmd_vel_out` topic

## Troubleshooting

1. **GPIO permissions:**
```bash
sudo usermod -a -G gpio $USER
# Logout and login again
```

2. **Check GPIO states:**
```bash
# Install gpio tools
sudo apt install gpiod
# Check pin states
gpioget gpiochip0 5 6 13 19 20 16 21 26
```

3. **Node not receiving messages:**
- Check topic name: `ros2 topic list | grep cmd_vel`
- Verify message type: `ros2 topic info /vaccum_base_controller/cmd_vel_out`

4. **Clean shutdown:**
The node automatically stops all motors and cleans up GPIO on shutdown (Ctrl+C).

## Safety Features

- **Velocity deadzone**: Motors stop when velocity is below threshold
- **Emergency stop**: All motors can be stopped immediately
- **GPIO cleanup**: Proper cleanup on node shutdown
- **Error handling**: Robust error handling for GPIO operations

## Motor Control Logic

The node converts `cmd_vel` (linear.x, linear.y, angular.z) into individual motor velocities:

```
For differential drive:
v_left = linear_x - (angular_z * wheel_separation / 2)
v_right = linear_x + (angular_z * wheel_separation / 2)

For mecanum drive (if linear.y != 0):
motor1 (FL) = v_left + linear_y
motor2 (FR) = v_right - linear_y  
motor3 (RL) = v_left - linear_y
motor4 (RR) = v_right + linear_y
```

## License

MIT License - see package.xml for details.

---

## Keyboard Control Node

The `keyboard_control` node provides a simple keyboard interface to publish integer values to 4 different ROS topics.

### Key Mappings

The node monitors 4 groups of 3 keys each:

- **Group 1**: `q` (1), `a` (0), `z` (-1) → `/keyboard/group1`
- **Group 2**: `w` (1), `s` (0), `x` (-1) → `/keyboard/group2`  
- **Group 3**: `r` (1), `f` (0), `v` (-1) → `/keyboard/group3`
- **Group 4**: `t` (1), `g` (0), `b` (-1) → `/keyboard/group4`

For each group:
- First key publishes `1`
- Middle key publishes `0` 
- Last key publishes `-1`

### Running the Keyboard Control Node

1. **Direct execution:**
```bash
ros2 run rpi_nodes keyboard_control
```

2. **Using launch file:**
```bash
ros2 launch rpi_nodes keyboard_control.launch.py
```

3. **Test the keyboard input:**
```bash
# In terminal 1 - run the keyboard node
ros2 run rpi_nodes keyboard_control

# In terminal 2 - monitor topics
ros2 topic echo /keyboard/group1
# OR run the test script
python3 src/rpi_nodes/scripts/test_keyboard.py
```

### Usage Notes

- The node requires terminal input, so it must be run in an interactive terminal
- Press `ESC` key to quit the node
- The node uses raw terminal input for immediate key response
- All keys are case-insensitive

### Example Output

```
[INFO] [keyboard_control_node]: Key "q" pressed -> group1: 1
[INFO] [keyboard_control_node]: Key "s" pressed -> group2: 0  
[INFO] [keyboard_control_node]: Key "v" pressed -> group3: -1
```
