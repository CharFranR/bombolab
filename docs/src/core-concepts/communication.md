# Serial Communication

**Hardware interface for sending joint angles to the Arduino Nano firmware.**

The `communication` module provides a serial connection to the robot's microcontroller, sending interpolated angle commands and verifying responses.

## Architecture

```
communication/
├── mod.rs              # ConnectionError, protocol constants
├── arduino_nano.rs     # ArduinoNano serial wrapper
└── interpolation.rs    # Smooth multi-joint movement
```

## Quick Start

```rust
use bombolab_core::communication::{ArduinoNano, InterpolationConfig, interpolate_all};

// Connect to Arduino
let mut nano = ArduinoNano::connect("/dev/ttyUSB0")?;

// Send angles directly (6 values: 5 joints + gripper)
nano.send_and_verify(&[90, 115, 110, 170, 90, 90])?;

// Or interpolate smoothly between positions
let current = [90, 115, 110, 170, 90, 90];
let target = [90, 120, 100, 170, 90, 90];
let config = InterpolationConfig::default();

let steps = interpolate_all(&current, &target, &config);
for step in &steps {
    nano.send_and_verify(step)?;
    std::thread::sleep(std::time::Duration::from_millis(config.delay_ms));
}

nano.disconnect()?;
```

## Serial Protocol

### Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `BAUD_RATE` | 115200 | Serial baud rate |
| `JOINT_COUNT` | 6 | Values per message (5 joints + gripper) |
| `ANGLE_MIN` | 10 | Minimum servo angle (degrees) |
| `ANGLE_MAX` | 170 | Maximum servo angle (degrees) |
| `READ_TIMEOUT_MS` | 1000 | Read timeout (milliseconds) |

### Message Format

```
TX: "90,115,110,170,90,90\n"
RX: "OK\n"
```

The 6 values map to:

| Index | Servo | Joint | Description |
|-------|-------|-------|-------------|
| 0 | S1 | J1 | Base yaw |
| 1 | S2 | J2 | Shoulder pitch |
| 2 | S3 | J3 | Elbow pitch |
| 3 | S4 | J4 | Wrist roll |
| 4 | S5 | J5 | Wrist pitch |
| 5 | S6 | — | Gripper |

**Note:** The gripper (index 5) is not part of the 5-DOF kinematic model but is included in the serial protocol for hardware control.

## ArduinoNano

### Connection

```rust
use bombolab_core::communication::ArduinoNano;

// List available ports
let ports = ArduinoNano::list_ports()?;

// Connect
let mut nano = ArduinoNano::connect("/dev/ttyUSB0")?;

// Send angles and verify response
nano.send_and_verify(&[90, 115, 110, 170, 90, 90])?;

// Clean disconnect (also handled by Drop)
nano.disconnect()?;
```

### Methods

| Method | Description |
|--------|-------------|
| `list_ports()` | Returns available serial port names |
| `connect(port_name)` | Opens connection at 115200 baud |
| `send_angles(angles)` | Sends 6 angles as CSV |
| `read_response()` | Reads one line (expects "OK" or "ERR") |
| `send_and_verify(angles)` | Send + read + check for "OK" |
| `disconnect()` | Flush and close port |

### Error Handling

All methods return `Result<T, ConnectionError>`:

```rust
use bombolab_core::communication::ConnectionError;

match nano.send_and_verify(&angles) {
    Ok(()) => println!("Angles sent"),
    Err(ConnectionError::InvalidResponse { port, response }) => {
        eprintln!("Arduino returned: {}", response);
    }
    Err(ConnectionError::WriteFailed { port, source }) => {
        eprintln!("Serial write failed: {}", source);
    }
    Err(e) => eprintln!("Error: {}", e),
}
```

## Interpolation

Moving directly from one angle to another can cause jerky motion. The interpolation module generates smooth intermediate steps.

### Configuration

```rust
use bombolab_core::communication::InterpolationConfig;

let config = InterpolationConfig {
    step_size: 5,   // degrees per step
    delay_ms: 40,   // milliseconds between steps
};
```

### Single Joint

```rust
use bombolab_core::communication::interpolate_joint;

// 90° → 100° with step 5°
let steps = interpolate_joint(90, 100, 5);
assert_eq!(steps, vec![95, 100]);

// Non-exact step: 90° → 102° with step 5°
let steps = interpolate_joint(90, 102, 5);
assert_eq!(steps, vec![95, 100, 102]);
```

### All Joints

```rust
use bombolab_core::communication::interpolate_all;

let current = [90, 90, 90, 90, 90, 90];
let target = [95, 100, 85, 115, 90, 90];
let config = InterpolationConfig { step_size: 5, delay_ms: 0 };

let steps = interpolate_all(&current, &target, &config);

// All joints move simultaneously:
// - Faster joints pad with their final value
// - All arrive at target in the same number of steps
assert_eq!(steps.len(), 5); // J3 (90→115) dictates step count
```

**Key behavior:**
- Each joint interpolates independently
- Shorter movements pad with their final value
- All joints arrive at the target simultaneously

## CLI Tool: serial-test

An interactive REPL for testing serial communication:

```bash
cargo run --bin serial-test
```

### Commands

```
> 1 90        # Move servo 1 to 90°
> all 90 115 110 170 90 90  # Move all servos
> quit        # Disconnect and exit
```

### Features

- Auto-detects USB serial devices (`/dev/ttyUSB*`, `/dev/ttyACM*`)
- Ctrl+C handler for clean disconnect
- Angle validation (10–170° range)
- Smooth interpolation between positions

## Integration with Robot Model

To send robot joint angles to hardware:

```rust
use bombolab_core::{fabri_creator, communication::{ArduinoNano, interpolate_all}};

let robot = fabri_creator();
let mut nano = ArduinoNano::connect("/dev/ttyUSB0")?;

// Get kinematic angles from robot (in q-space)
let q = vec![0.1, -0.2, 0.3, -0.1, 0.15];

// Convert to servo angles
let servo = robot.q_to_servo(&q);

// Add gripper value (6th element)
let mut angles: Vec<i32> = servo.iter()
    .map(|s| (s.to_degrees() as i32).clamp(10, 170))
    .collect();
angles.push(90); // gripper default

let target: [i32; 6] = angles.try_into().unwrap();

// Interpolate and send
let current = [90, 115, 110, 170, 90, 90]; // or track actual position
let steps = interpolate_all(&current, &target, &Default::default());

for step in &steps {
    nano.send_and_verify(step)?;
    std::thread::sleep(std::time::Duration::from_millis(40));
}
```

## Testing

```bash
# Run communication module tests
cargo test -p bombolab-core communication

# Run interpolation tests specifically
cargo test -p bombolab-core interpolation
```

The interpolation tests cover:
- Exact multiples, non-exact steps, no movement
- Ascending and descending angles
- Multi-joint padding and synchronization

## Troubleshooting

### Port not found

```
Error: port not found: /dev/ttyUSB0
```

**Causes:**
- Arduino not plugged in
- Wrong port name (check with `ls /dev/ttyUSB* /dev/ttyACM*`)
- Permission issue (user not in `dialout` group)

**Fix:**

```bash
# Check available ports
ls /dev/ttyUSB* /dev/ttyACM*

# Add yourself to the dialout group (Linux)
sudo usermod -a -G dialout $USER
# Then log out and back in
```

### Permission denied

```
Error: failed to open /dev/ttyUSB0: Permission denied
```

**Fix:** Add your user to the `dialout` group:

```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect
```

### Arduino not responding

```
Error: invalid response from /dev/ttyUSB0: ""
```

**Causes:**
- Arduino firmware not uploaded
- Wrong baud rate (must be 115200)
- Arduino reset (unplug and replug)

**Fix:**
1. Verify firmware is uploaded: open Arduino IDE Serial Monitor at 115200 baud, type `90,115,110,170,90,90` and press Enter. You should see `OK`.
2. Check baud rate matches: `BAUD_RATE` in `communication/mod.rs` must match the firmware's `Serial.begin()` value.

### Servos jittering or moving erratically

**Causes:**
- Insufficient power supply (USB power may not be enough for all servos)
- Angles outside valid range
- Serial data corruption

**Fix:**
1. Use an external 5V power supply for servos (not USB power)
2. Ensure angles are between 10° and 170°
3. Check serial cable length — keep under 1 meter for reliable communication

### Intermittent ERR responses

```
Error: invalid response from /dev/ttyUSB0: "ERR"
```

**Causes:**
- Angle out of range (firmware validates 10–170°)
- Mechanical obstruction preventing servo movement

**Fix:**
1. Check which joint caused the error (log the angles being sent)
2. Verify all angles are within 10–170°
3. Move the robot manually to check for mechanical binding

### Timeout waiting for response

```
Error: read timeout on /dev/ttyUSB0 after 1000ms
```

**Causes:**
- Arduino frozen or crashed
- Serial buffer overflow (sending too fast)
- USB connection unstable

**Fix:**
1. Reset Arduino (unplug and replug)
2. Increase delay between commands (currently 40ms default)
3. Try a different USB port or cable
