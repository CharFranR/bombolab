# serial-test

**Interactive CLI for testing serial communication with the Arduino Nano.**

## Usage

```bash
cargo run --bin serial-test
```

## Features

- Auto-detects USB serial devices (`/dev/ttyUSB*`, `/dev/ttyACM*`)
- Interactive REPL for sending angle commands
- Smooth interpolation between positions
- Clean Ctrl+C shutdown

## Example Session

```
=== Serial Test — Arduino Nano ===

Found USB device: /dev/ttyUSB0
Connected to /dev/ttyUSB0

Commands:
  <servo> <angle>       Move single servo (1-6, angle 10-170)
  all <a1> <a2> ... <a6>  Move all servos
  quit / exit           Disconnect and exit

> 1 90
Moving 1 steps...
Done.
> all 90 120 100 170 90 90
Moving 5 steps...
Done.
> quit
Disconnecting...
Done.
```

## Commands

| Command | Description |
|---------|-------------|
| `<servo> <angle>` | Move single servo (1-6) to angle (10-170°) |
| `all <a1> <a2> ... <a6>` | Move all 6 servos to specified angles |
| `quit` / `exit` | Disconnect and exit |
| `Ctrl+C` | Clean disconnect and exit |

## Port Detection

The tool automatically filters for USB serial devices:
- `/dev/ttyUSB*` — FTDI-based adapters
- `/dev/ttyACM*` — Native USB devices (Arduino Nano, etc.)

If multiple devices are found, you'll be prompted to select one.

## See Also

- [Serial Communication](../core-concepts/communication.md) — module documentation
- [FABRI Creator](../core-concepts/fabri-creator.md) — robot configuration
