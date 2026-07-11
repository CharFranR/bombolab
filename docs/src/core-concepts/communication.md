# Serial Communication

**Hardware interface for sending joint angles to the Arduino Nano firmware.**

The `communication` module provides a serial connection to the robot's microcontroller, sending interpolated angle commands and verifying responses.

## Architecture

```
communication/
├── mod.rs              # ConnectionError, protocol constants, pin mapping
├── arduino_nano.rs     # ArduinoNano serial wrapper
├── command.rs          # ServoCommand typed struct
├── mapper.rs           # ServoMapper — centralized q→servo mapping
└── interpolation.rs    # Smooth multi-joint movement
```

## Quick Start

```rust
use bombolab_core::communication::{
    ArduinoNano, ServoCommand, InterpolationConfig, interpolate_all,
};

// Connect to Arduino
let mut nano = ArduinoNano::connect("/dev/ttyUSB0")?;

// Send angles via ServoCommand (typed struct: 5 joints + gripper)
let cmd = ServoCommand::new([90.0, 115.0, 110.0, 170.0, 90.0], 90)?;
nano.send(&cmd)?;

// Or interpolate smoothly between positions
let current = [90, 115, 110, 170, 90, 90];
let target = [90, 120, 100, 170, 90, 90];
let config = InterpolationConfig::default();

let steps = interpolate_all(&current, &target, &config);
for step in &steps {
    let cmd = ServoCommand::from_raw_array(step);
    nano.send(&cmd)?;
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
use bombolab_core::communication::{ArduinoNano, ServoCommand};

// List available ports
let ports = ArduinoNano::list_ports()?;

// Connect
let mut nano = ArduinoNano::connect("/dev/ttyUSB0")?;

// Build a typed command and send it
let cmd = ServoCommand::new([90.0, 115.0, 110.0, 170.0, 90.0], 90)?;
nano.send(&cmd)?;

// Or send and wait for OK/ERR response
nano.send_and_verify(&cmd)?;

// Clean disconnect (also handled by Drop)
nano.disconnect()?;
```

### Methods

| Method | Description |
|--------|-------------|
| `list_ports()` | Returns available serial port names |
| `connect(port_name)` | Opens connection at 115200 baud |
| `send(cmd)` | Sends a `ServoCommand` as CSV |
| `send_and_verify(cmd)` | Send + read + check for "OK" |
| `read_response()` | Reads one line (expects "OK" or "ERR") |
| `disconnect()` | Flush and close port |

### Error Handling

All methods return `Result<T, ConnectionError>`:

```rust
use bombolab_core::communication::ConnectionError;

match nano.send_and_verify(&cmd) {
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

## ServoCommand

`ServoCommand` is a typed struct that replaces raw `[i32; 6]` arrays with semantic fields:

```rust
pub struct ServoCommand {
    pub joints: [f64; 5],  // 5 joint angles in degrees
    pub gripper: u8,        // gripper angle 0–255
}
```

| Method | Description |
|--------|-------------|
| `new(joints, gripper)` | Creates a `ServoCommand`, validates joints [10°, 170°] and gripper 0–255 |
| `to_wire()` | Serializes to `"a1,a2,a3,a4,a5,g\n"` for the serial protocol |
| `to_raw_array()` | Converts to `[i32; 6]` for use with `interpolate_all()` |
| `from_raw_array(arr)` | Builds from an `[i32; 6]` (e.g., from interpolation output) |

The wire format is identical to the previous raw-array API — no protocol breakage.

## ServoMapper

`ServoMapper` centralizes the conversion from kinematic coordinates (q in radians) to servo angles (degrees), combining offsets, clamping, and gripper into a single step:

```rust
use bombolab_core::{fabri_creator, communication::ServoMapper};

let robot = fabri_creator();
let mapper = ServoMapper::new(&robot);

// q = [0,0,0,0,0] → servo angles at home pose
let cmd = mapper.map_q(&[0.0; 5], 90);
assert_eq!(cmd.to_wire(), "90,115,110,170,90,90\n");
```

**What `ServoMapper` does internally:**

1. Delegates to `Robot::q_to_servo()` to add per-joint offsets (radians)
2. Converts radians to degrees
3. Clamps each joint to [10°, 170°]
4. Packages everything into a `ServoCommand`

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

The `ServoMapper` bridges kinematic q (radians) to servo angles (degrees), centralizing offset and clamping logic:

```rust
use bombolab_core::{
    fabri_creator,
    communication::{ArduinoNano, ServoMapper, ServoCommand, interpolate_all_command},
};

let robot = fabri_creator();
let mapper = ServoMapper::new(&robot);
let mut nano = ArduinoNano::connect("/dev/ttyUSB0")?;

// Map kinematic q to ServoCommand
let q = vec![0.1, -0.2, 0.3, -0.1, 0.15];
let target_cmd = mapper.map_q(&q, 90); // 90 = gripper angle

// Interpolate smoothly from home to target
let home = ServoCommand::new([90.0, 115.0, 110.0, 170.0, 90.0], 90)?;
let steps = interpolate_all_command(&home, &target_cmd, &Default::default());

for step in &steps {
    nano.send(&step)?;
    std::thread::sleep(std::time::Duration::from_millis(40));
}
```

No manual rad→deg conversion, clamping, or gripper appending needed — `ServoMapper` handles it all.

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
