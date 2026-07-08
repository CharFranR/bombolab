use bombolab_core::inverse_kinematics;
use bombolab_core::kinematics::dh::{solve, DHParameter};
use bombolab_core::kinematics::forward::forward_kinematics;
use bombolab_core::math::{Iso3, RAD_TO_DEG};
use bombolab_core::robot::fabri_creator::{base_transform, fabri_creator};
use bombolab_core::IkOptions;
use nalgebra::{Translation3, UnitQuaternion};

fn rpy_to_rotation(roll: f64, pitch: f64, yaw: f64) -> UnitQuaternion<f64> {
    let rx = nalgebra::Rotation3::from_axis_angle(&nalgebra::Vector3::x_axis(), roll);
    let ry = nalgebra::Rotation3::from_axis_angle(&nalgebra::Vector3::y_axis(), pitch);
    let rz = nalgebra::Rotation3::from_axis_angle(&nalgebra::Vector3::z_axis(), yaw);
    UnitQuaternion::from_rotation_matrix(&(rz * ry * rx))
}

/// Build a DHParameter table from robot segments, setting theta = q[i].
fn build_dh_table(robot: &bombolab_core::robot::Robot, q: &[f64]) -> Vec<DHParameter> {
    robot
        .segments
        .iter()
        .zip(q.iter())
        .map(|(seg, &qi)| {
            let (_theta, d, a, alpha) = seg.dh_params();
            // Override theta with the current joint angle
            DHParameter::new(alpha, a, d, qi)
        })
        .collect()
}

fn print_fk(robot: &bombolab_core::robot::Robot, base: &Iso3, label: &str) {
    let (frames, effector) = forward_kinematics(*base, robot);
    let pos = effector.translation.vector;
    println!("  {}: ({:.1}, {:.1}, {:.1}) mm", label, pos.x, pos.y, pos.z);
    if frames.len() <= 6 {
        for (i, f) in frames.iter().enumerate() {
            let fp = f.translation.vector;
            println!("    Frame {}: ({:.1}, {:.1}, {:.1})", i + 1, fp.x, fp.y, fp.z);
        }
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();

    let is_interactive = args.len() < 2;

    if is_interactive || args[1] == "--help" || args[1] == "-h" {
        eprintln!("Uso: cargo run --bin ik-solve <x_mm> <y_mm> <z_mm> [roll_deg] [pitch_deg] [yaw_deg]");
        eprintln!();
        eprintln!("Resuelve cinemática inversa para el FABRI Creator 5-DOF.");
        eprintln!("Los ángulos de orientación (roll, pitch, yaw) son respecto a la base.");
        eprintln!("Si no se especifica orientación, la rotación queda libre (solo Z-roll).");
        eprintln!();
        eprintln!("Ejemplos:");
        eprintln!("  cargo run --bin ik-solve 200 0 150");
        eprintln!("  cargo run --bin ik-solve 180 50 120 0 -45 0");
        std::process::exit(1);
    }

    let x: f64 = args[1].parse().expect("x debe ser un número (mm)");
    let y: f64 = args[2].parse().expect("y debe ser un número (mm)");
    let z: f64 = args[3].parse().expect("z debe ser un número (mm)");

    let (roll_deg, pitch_deg, yaw_deg, has_orientation) = if args.len() >= 7 {
        (
            Some(args[4].parse::<f64>().unwrap_or(0.0)),
            Some(args[5].parse::<f64>().unwrap_or(0.0)),
            Some(args[6].parse::<f64>().unwrap_or(0.0)),
            true,
        )
    } else {
        (None, None, None, false)
    };

    // ── Build robot ──────────────────────────────────────────────
    let robot = fabri_creator();
    let base = base_transform();

    // ── Show FK at home pose for reference ────────────────────────
    println!("=== IK Solver — FABRI Creator 5-DOF ===\n");
    println!("FK at home pose (q=[0,0,0,0,0]):");
    print_fk(&robot, &base, "End effector");
    println!();

    // ── Determine target orientation ─────────────────────────────
    // If orientation is specified, use RPY; otherwise use FK at seed
    // so the solver only has to solve for position, not orientation.
    let q_seed = robot.kinematic_home();
    let seed_fk = {
        let dh_table = build_dh_table(&robot, &q_seed);
        let sol = solve(&dh_table);
        sol.final_transform
    };

    let target = if has_orientation {
        let rotation = rpy_to_rotation(
            roll_deg.unwrap().to_radians(),
            pitch_deg.unwrap().to_radians(),
            yaw_deg.unwrap().to_radians(),
        );
        Iso3::from_parts(Translation3::new(x, y, z), rotation)
    } else {
        // Use FK at seed orientation + target position
        let rot_mat: nalgebra::Matrix3<f64> =
            seed_fk.fixed_view::<3, 3>(0, 0).into_owned();
        let seed_rot = UnitQuaternion::from_matrix(&rot_mat);
        Iso3::from_parts(Translation3::new(x, y, z), seed_rot)
    };

    // ── Run IK ──────────────────────────────────────────────────
    let opts = IkOptions::default();

    println!("Target position: ({:.1}, {:.1}, {:.1}) mm", x, y, z);
    if has_orientation {
        println!(
            "Target orientation: roll={:.1}°, pitch={:.1}°, yaw={:.1}°",
            roll_deg.unwrap(),
            pitch_deg.unwrap(),
            yaw_deg.unwrap()
        );
    } else {
        println!("Target orientation: (usa orientación de seed — solo resuelve posición)");
    }
    println!(
        "Seed: q = [{:.2}, {:.2}, {:.2}, {:.2}, {:.2}] (home pose)\n",
        q_seed[0], q_seed[1], q_seed[2], q_seed[3], q_seed[4]
    );

    match inverse_kinematics(&robot, &base, &target, &q_seed, &opts) {
        Ok(result) => {
            println!("── Result ──────────────────────────────────────");
            println!("  Converged:     {}", if result.converged { "YES ✓" } else { "NO ✗" });
            println!("  Iterations:    {}", result.iterations);
            println!("  Error pos:     {:.4} mm", result.error_pos);
            println!("  Error angle:   {:.4} rad", result.error_angle);
            println!();

            println!("  Joint angles (kinematic q — rad / deg):");
            for (i, &q) in result.q.iter().enumerate() {
                println!("    J{}: {:.6} rad  ({:.2}°)", i + 1, q, q * RAD_TO_DEG);
            }
            println!();
            let servo = robot.q_to_servo(&result.q);
            println!("  Joint angles (servo — rad / deg):");
            for (i, &s) in servo.iter().enumerate() {
                println!("    S{}: {:.6} rad  ({:.2}°)", i + 1, s, s * RAD_TO_DEG);
            }

            // Round-trip: FK(IK) → where did we end up?
            let dh_table = build_dh_table(&robot, &result.q);
            let fk_solution = solve(&dh_table);
            let fk_pos = fk_solution.translation();
            println!();
            println!("── Round-trip FK(IK) ──────────────────────────");
            println!(
                "  End effector: ({:.2}, {:.2}, {:.2}) mm",
                fk_pos.x, fk_pos.y, fk_pos.z
            );
            let dx = fk_pos.x - x;
            let dy = fk_pos.y - y;
            let dz = fk_pos.z - z;
            println!(
                "  Delta:        ({:.4}, {:.4}, {:.4}) mm",
                dx, dy, dz
            );
            println!("  Total error:  {:.4} mm", (dx * dx + dy * dy + dz * dz).sqrt());

            // Check joint limits
            println!();
            println!("── Joint limits ────────────────────────────────");
            for (i, seg) in robot.segments.iter().enumerate() {
                let q = result.q[i];
                let within = q >= seg.joint.value_min && q <= seg.joint.value_max;
                println!(
                    "  J{}: {:.4} rad  [{:.1}°, {:.1}°] {}",
                    i + 1,
                    q,
                    seg.joint.value_min * RAD_TO_DEG,
                    seg.joint.value_max * RAD_TO_DEG,
                    if within { "✓" } else { "✗ OUT OF RANGE" }
                );
            }

            if !result.converged {
                println!();
                println!("  ⚠ El solver no convergió. El target puede estar fuera del alcance.");
                println!("     Probá con un seed diferente o un target más cercano.");
            }
        }
        Err(e) => {
            println!("── Error ────────────────────────────────────────");
            println!("  IK solver failed: {}", e);
        }
    }
}
