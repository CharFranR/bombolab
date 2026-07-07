use std::io::{self, Write};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

use bombolab_core::communication::{
    ANGLE_MAX, ANGLE_MIN, ArduinoNano, InterpolationConfig, JOINT_COUNT, interpolate_all,
};

fn read_input(prompt: &str) -> String {
    print!("{}", prompt);
    io::stdout().flush().unwrap();
    let mut input = String::new();
    io::stdin().read_line(&mut input).unwrap();
    input.trim().to_string()
}

fn main() {
    println!("=== Serial Test — Arduino Nano ===\n");

    // List available ports
    let ports = match ArduinoNano::list_ports() {
        Ok(p) => p,
        Err(e) => {
            eprintln!("Error listing ports: {}", e);
            return;
        }
    };

    if ports.is_empty() {
        println!("No serial ports found. Connect Arduino and try again.");
        return;
    }

    // Filter USB/ACM ports (ttyUSB for FTDI, ttyACM for native USB like Arduino Nano)
    let usb_ports: Vec<String> = ports
        .iter()
        .filter(|p| p.starts_with("/dev/ttyUSB") || p.starts_with("/dev/ttyACM"))
        .cloned()
        .collect();

    let port_name = if usb_ports.len() == 1 {
        println!("Found USB device: {}", usb_ports[0]);
        usb_ports[0].clone()
    } else if usb_ports.is_empty() {
        eprintln!("No USB serial devices found. Connect Arduino and try again.");
        return;
    } else {
        println!("USB devices:");
        for (i, port) in usb_ports.iter().enumerate() {
            println!("  [{}] {}", i + 1, port);
        }
        loop {
            let input = read_input("\nPort #: ");
            match input.parse::<usize>() {
                Ok(n) if n >= 1 && n <= usb_ports.len() => break usb_ports[n - 1].clone(),
                _ => println!(
                    "Invalid selection. Enter a number between 1 and {}.",
                    usb_ports.len()
                ),
            }
        }
    };

    // Connect
    let nano = match ArduinoNano::connect(&port_name) {
        Ok(n) => {
            println!("Connected to {}", port_name);
            n
        }
        Err(e) => {
            eprintln!("Connection failed: {}", e);
            return;
        }
    };

    let nano = Arc::new(Mutex::new(Some(nano)));
    let running = Arc::new(AtomicBool::new(true));

    // Ctrl+C handler: disconnect cleanly and signal loop to exit
    {
        let nano = nano.clone();
        let running = running.clone();
        ctrlc::set_handler(move || {
            running.store(false, Ordering::SeqCst);
            if let Ok(mut guard) = nano.lock() {
                if let Some(ref mut n) = *guard {
                    let _ = n.disconnect();
                }
                *guard = None;
            }
            eprintln!("\nInterrupted. Disconnected.");
        })
        .expect("Error setting Ctrl+C handler");
    }

    // REPL
    let config = InterpolationConfig::default();
    let mut current_angles = [90i32; JOINT_COUNT];

    println!("\nCommands:");
    println!("  <servo> <angle>       Move single servo (1-6, angle 10-170)");
    println!("  all <a1> <a2> ... <a6>  Move all servos");
    println!("  quit / exit           Disconnect and exit\n");

    loop {
        if !running.load(Ordering::SeqCst) {
            break;
        }

        let input = read_input("> ");
        let parts: Vec<&str> = input.split_whitespace().collect();

        match parts.first().map(|s| *s) {
            Some("quit") | Some("exit") => {
                println!("Disconnecting...");
                if let Ok(mut guard) = nano.lock() {
                    if let Some(ref mut n) = *guard {
                        let _ = n.disconnect();
                    }
                    *guard = None;
                }
                break;
            }
            Some("all") if parts.len() == JOINT_COUNT + 1 => {
                let mut target = [0i32; JOINT_COUNT];
                let mut valid = true;
                for i in 0..JOINT_COUNT {
                    match parts[i + 1].parse::<i32>() {
                        Ok(v) if (ANGLE_MIN..=ANGLE_MAX).contains(&v) => target[i] = v,
                        _ => {
                            eprintln!(
                                "Invalid angle for joint {}: '{}' (must be {}-{})",
                                i + 1,
                                parts[i + 1],
                                ANGLE_MIN,
                                ANGLE_MAX
                            );
                            valid = false;
                            break;
                        }
                    }
                }
                if valid {
                    let mut guard = nano.lock().unwrap();
                    if let Some(ref mut n) = *guard {
                        execute_movement(n, &mut current_angles, &target, &config);
                    }
                }
            }
            Some(s) if s.parse::<usize>().is_ok() => {
                let servo: usize = s.parse().unwrap();
                if servo < 1 || servo > JOINT_COUNT {
                    eprintln!("Servo must be 1-{}", JOINT_COUNT);
                    continue;
                }
                let angle = match parts.get(1).and_then(|a| a.parse::<i32>().ok()) {
                    Some(a) if (ANGLE_MIN..=ANGLE_MAX).contains(&a) => a,
                    _ => {
                        eprintln!(
                            "Invalid angle: {:?} (must be {}-{})",
                            parts.get(1),
                            ANGLE_MIN,
                            ANGLE_MAX
                        );
                        continue;
                    }
                };
                let mut target = current_angles;
                target[servo - 1] = angle;
                let mut guard = nano.lock().unwrap();
                if let Some(ref mut n) = *guard {
                    execute_movement(n, &mut current_angles, &target, &config);
                }
            }
            _ if !input.is_empty() => {
                println!("Unknown command. Type 'quit' to exit.");
            }
            _ => {}
        }
    }

    println!("Done.");
}

fn execute_movement(
    nano: &mut ArduinoNano,
    current: &mut [i32; JOINT_COUNT],
    target: &[i32; JOINT_COUNT],
    config: &InterpolationConfig,
) {
    let steps = interpolate_all(current, target, config);
    if steps.is_empty() {
        println!("Already at target.");
        return;
    }

    println!("Moving {} steps...", steps.len());
    for (i, step) in steps.iter().enumerate() {
        if let Err(e) = nano.send_and_verify(step) {
            eprintln!("Error at step {}: {}", i + 1, e);
            eprintln!("Stopping movement.");
            return;
        }
        std::thread::sleep(std::time::Duration::from_millis(config.delay_ms));
    }
    *current = *target;
    println!("Done.");
}
