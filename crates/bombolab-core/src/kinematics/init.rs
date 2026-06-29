use std::io::{self, Write};

use nalgebra::{Isometry3, Vector3};

use crate::math::hmatrix::{Movement, make_movement};

use crate::kinematics::forward::forward_kinematics;
use crate::robot::{DHParams, Joint, JointType, Result, Robot, Segment};

fn read_vec3(prompt: &str) -> Vector3<f64> {
    print!("{}", prompt);
    io::stdout().flush().unwrap();
    let mut input = String::new();
    io::stdin().read_line(&mut input).unwrap();
    let parts: Vec<f64> = input
        .trim()
        .split_whitespace()
        .map(|s| s.parse().unwrap())
        .collect();
    Vector3::new(parts[0], parts[1], parts[2])
}

fn read_f64(prompt: &str) -> f64 {
    print!("{}", prompt);
    io::stdout().flush().unwrap();
    let mut input = String::new();
    io::stdin().read_line(&mut input).unwrap();
    input.trim().parse().unwrap()
}

fn read_i32(prompt: &str) -> i32 {
    print!("{}", prompt);
    io::stdout().flush().unwrap();
    let mut input = String::new();
    io::stdin().read_line(&mut input).unwrap();
    input.trim().parse().unwrap()
}

fn read_joint_type(prompt: &str) -> JointType {
    loop {
        print!("{} (0 = Revolute, 1 = Prismatic): ", prompt);
        io::stdout().flush().unwrap();
        let mut input = String::new();
        io::stdin().read_line(&mut input).unwrap();
        match input.trim() {
            "0" => return JointType::Revolute,
            "1" => return JointType::Prismatic,
            _ => println!("Invalid option, enter 0 or 1."),
        }
    }
}

fn read_movement(n: usize) -> Movement {
    println!("\n--- Movement {} ---", n);
    let translation = read_vec3("  Translation (x y z): ");
    let axis = read_vec3("  Rotation axis (x y z): ");
    let degrees = read_f64("  Angle (degrees): ");
    let isometry = read_f64("  Order (1 = rot+trans, 0 = trans+rot): ") != 0.0;

    Movement {
        translation,
        angles: degrees.to_radians(),
        axis,
        isometry,
    }
}

pub fn run() {
    println!("=== make_movement test ===\n");

    let initial_pos = read_vec3("Initial effector position (x y z): ");
    let initial = Isometry3::new(initial_pos, nalgebra::zero());

    let mov1 = read_movement(1);
    let mov2 = read_movement(2);

    let (trajectory, final_pos) = make_movement(initial, &[mov1, mov2]);

    println!("\n=== Results ===");
    for (i, pose) in trajectory.iter().enumerate() {
        let p = pose.translation.vector;
        println!(
            "  After movement {}: ({:.4}, {:.4}, {:.4})",
            i + 1,
            p.x,
            p.y,
            p.z
        );
    }

    let fp = final_pos.translation.vector;
    println!(
        "\n  Final effector position: ({:.4}, {:.4}, {:.4})",
        fp.x, fp.y, fp.z
    );
}

pub fn make_robot() -> Robot {
    println!("\n=== Let's build the robot ===\n");
    let num_segments = read_i32("Number of segments: ") as usize;

    let mut segments: Vec<Segment> = Vec::new();

    for i in 0..num_segments {
        println!("\nSegment {}", i + 1);
        let joint_type = read_joint_type("Joint type");
        let value = read_f64("  Angle (degrees): ").to_radians();
        let value_max = read_f64("  Max angle (degrees): ").to_radians();
        let value_min = read_f64("  Min angle (degrees): ").to_radians();

        println!("DH Parameters");
        let theta = read_f64("  Theta (degrees): ").to_radians();
        let alpha = read_f64("  Alpha (degrees): ").to_radians();
        let d = read_f64("  d: ");
        let a = read_f64("  a: ");

        let new_joint = Joint::new(joint_type, value, value_max, value_min);
        let new_link = DHParams::new(theta, d, a, alpha);

        segments.push(Segment::new(new_joint, new_link));
    }

    Robot::new(segments)
}

pub fn move_robot(robot: &mut Robot) -> Result<()> {
    println!("\nLet's move the robot\n");

    for i in 0..robot.dof() {
        println!("--- Joint {} ---", i + 1);
        let segment = robot.segment_mut(i)?;
        let old_val = segment.joint.value.to_degrees();
        let new_val = read_f64(&format!(
            "  New angle (degrees) [current: {:.2}]: ",
            old_val
        ));
        segment.joint.value = new_val.to_radians();
    }

    println!("\nMovement completed");

    Ok(())
}

pub fn test_forward_kinematics() {
    let mut pajatron = make_robot();

    let base = Isometry3::<f64>::identity();

    let (frames, effector) = forward_kinematics(base, &pajatron);

    println!("\nInitial positions");
    for (i, f) in frames.iter().enumerate() {
        let p = f.translation.vector;
        println!("  Frame {}: ({:.4}, {:.4}, {:.4})", i + 1, p.x, p.y, p.z);
    }
    let ep = effector.translation.vector;
    println!("End effector: ({:.4}, {:.4}, {:.4})", ep.x, ep.y, ep.z);

    if let Err(e) = move_robot(&mut pajatron) {
        println!("Error moving robot: {}", e);
        return;
    }

    let (_frames2, effector2) = forward_kinematics(base, &pajatron);
    let ep2 = effector2.translation.vector;
    println!(
        "\nEnd effector after moving: ({:.4}, {:.4}, {:.4})",
        ep2.x, ep2.y, ep2.z
    );
}
