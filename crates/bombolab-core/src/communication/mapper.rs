//! ServoMapper — centralized q→servo mapping with clamping.
//!
//! Converts kinematic coordinates (radians) to servo angles (degrees)
//! by delegating to `Robot::q_to_servo()`, then clamping to [5°, 175°]
//! and producing a `ServoCommand`.
//!
//! # Future direction inversion
//!
//! The design supports per-joint sign inversion (`servo = -q + offset`)
//! but no joint currently requires it. Add a `signs: [f64; 5]` field
//! to `ServoMapper` if needed.

use super::command::ServoCommand;
use crate::robot::Robot;

/// Maps kinematic joint angles (radians) to servo angles (degrees)
/// with clamping and gripper passthrough.
pub struct ServoMapper<'a> {
    robot: &'a Robot,
    angle_min: f64,
    angle_max: f64,
}

impl<'a> ServoMapper<'a> {
    /// Create a new mapper with default clamping [5°, 175°].
    pub fn new(robot: &'a Robot) -> Self {
        Self {
            robot,
            angle_min: super::ANGLE_MIN as f64,
            angle_max: super::ANGLE_MAX as f64,
        }
    }

    /// Map kinematic q (radians) to a `ServoCommand`.
    ///
    /// Delegates to `Robot::q_to_servo()` for rad→rad conversion,
    /// then converts to degrees and clamps to [`angle_min`, `angle_max`].
    ///
    /// # Errors
    ///
    /// Returns an error string if any mapped angle is non-finite (NaN/±Inf)
    /// or outside the accepted range after clamping.
    pub fn map_q(&self, q: &[f64], gripper: u8) -> Result<ServoCommand, &'static str> {
        let servo_rad = self.robot.q_to_servo(q);
        let mut joints = [0.0_f64; 5];
        for (i, &sr) in servo_rad.iter().enumerate().take(5) {
            let deg = sr.to_degrees();
            if !deg.is_finite() {
                return Err("joint angle out of range (non-finite)");
            }
            joints[i] = deg.clamp(self.angle_min, self.angle_max);
        }
        ServoCommand::new(joints, gripper)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::robot::joint::{Joint, JointType};
    use crate::robot::link::DHParams;
    use crate::robot::segment::{Robot, Segment};

    fn make_robot() -> Robot {
        let seg = || -> Segment {
            Segment::new(
                Joint::new(
                    JointType::Revolute,
                    0.0,
                    std::f64::consts::PI,
                    -std::f64::consts::PI,
                ),
                DHParams::new(0.0, 0.0, 0.0, 0.0),
            )
        };
        let offset = std::f64::consts::FRAC_PI_2; // 90°
        Robot::with_offsets(
            (0..5).map(|_| seg()).collect(),
            vec![offset; 5],
            vec![offset; 5],
        )
    }

    #[test]
    fn test_home_pose_maps_correctly() {
        let robot = make_robot();
        let mapper = ServoMapper::new(&robot);
        // q = [0,0,0,0,0] → servo = offsets = [90, 90, 90, 90, 90]
        let cmd = mapper.map_q(&[0.0; 5], 90).unwrap();
        assert!((cmd.joints[0] - 90.0).abs() < 1e-6);
        assert!((cmd.joints[1] - 90.0).abs() < 1e-6);
        assert!((cmd.joints[2] - 90.0).abs() < 1e-6);
        assert!((cmd.joints[3] - 90.0).abs() < 1e-6);
        assert!((cmd.joints[4] - 90.0).abs() < 1e-6);
        assert_eq!(cmd.gripper, 90);
    }

    #[test]
    fn test_clamp_below_min() {
        let robot = make_robot();
        let mapper = ServoMapper::new(&robot);
        // q value that maps to a very small servo angle (< 5°)
        // Offset J0 = 90° → q = -1.5 rad → servo ≈ -85.9° → clamped to 5°
        let cmd = mapper.map_q(&[-1.5, 0.0, 0.0, 0.0, 0.0], 90).unwrap();
        assert!((cmd.joints[0] - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_clamp_above_max() {
        let robot = make_robot();
        let mapper = ServoMapper::new(&robot);
        // q value that maps to a very large servo angle (> 175°)
        // Offset J0 = 90° → q = 1.6 rad → servo ≈ 181.7° → clamped to 175°
        let cmd = mapper.map_q(&[1.6, 0.0, 0.0, 0.0, 0.0], 90).unwrap();
        assert!((cmd.joints[0] - 175.0).abs() < 1e-6);
    }

    #[test]
    fn test_gripper_passthrough() {
        let robot = make_robot();
        let mapper = ServoMapper::new(&robot);
        let cmd = mapper.map_q(&[0.0; 5], 127).unwrap();
        assert_eq!(cmd.gripper, 127);
    }

    #[test]
    fn test_non_home_q_maps_with_offset() {
        let robot = make_robot();
        let mapper = ServoMapper::new(&robot);
        // q = [0.1, -0.2, 0.0, 0.15, -0.1] rad
        // All offsets = 90° (π/2 ≈ 1.571 rad)
        // servo_deg = (q + offset) * 180/π → clamped to [10, 170]
        let cmd = mapper.map_q(&[0.1, -0.2, 0.0, 0.15, -0.1], 45).unwrap();
        // J0: (0.1 + 1.571) rad * 180/π ≈ 95.73°
        // J1: (-0.2 + 1.571) rad * 180/π ≈ 78.56°
        // J2: (0.0 + 1.571) rad * 180/π ≈ 90.00°
        // J3: (0.15 + 1.571) rad * 180/π ≈ 98.60° — within limits, not clamped
        // J4: (-0.1 + 1.571) rad * 180/π ≈ 84.27°
        assert!((cmd.joints[0] - 95.73).abs() < 0.1);
        assert!((cmd.joints[1] - 78.56).abs() < 0.1);
        assert!((cmd.joints[2] - 90.00).abs() < 0.1);
        assert!((cmd.joints[3] - 98.60).abs() < 0.1);
        assert!((cmd.joints[4] - 84.27).abs() < 0.1);
    }

    #[test]
    fn test_output_is_servo_command() {
        let robot = make_robot();
        let mapper = ServoMapper::new(&robot);
        let cmd = mapper.map_q(&[0.1, -0.2, 0.3, -0.1, 0.15], 45).unwrap();
        // to_wire should produce valid output
        let wire = cmd.to_wire();
        assert!(wire.ends_with('\n'));
        assert_eq!(wire.split(',').count(), 6);
    }

    #[test]
    fn test_nan_q_rejected() {
        let robot = make_robot();
        let mapper = ServoMapper::new(&robot);
        assert!(mapper.map_q(&[f64::NAN, 0.0, 0.0, 0.0, 0.0], 90).is_err());
    }

    #[test]
    fn test_inf_q_rejected() {
        let robot = make_robot();
        let mapper = ServoMapper::new(&robot);
        assert!(
            mapper
                .map_q(&[f64::INFINITY, 0.0, 0.0, 0.0, 0.0], 90)
                .is_err()
        );
        assert!(
            mapper
                .map_q(&[f64::NEG_INFINITY, 0.0, 0.0, 0.0, 0.0], 90)
                .is_err()
        );
    }

    /// INVARIANTE de límites del FABRI real: la imagen de q_min/q_max del
    /// modelo debe caer EXACTAMENTE en [5°, 175°] — el rango que aceptan
    /// ServoCommand y el firmware. Si este test falla, el modelo promete
    /// configuraciones que el hardware no puede ejecutar (recorte silencioso).
    #[test]
    fn fabri_limits_map_exactly_to_servo_range() {
        let robot = crate::robot::fabri_creator();
        let mapper = ServoMapper::new(&robot);

        // q = límites inferiores → servo debe ser exactamente 5° (o el
        // límite superior del rango cuando la dirección es +1)
        let mut q_min = [0.0; 5];
        for (i, seg) in robot.segments.iter().enumerate() {
            q_min[i] = seg.joint.value_min;
        }
        let cmd_min = mapper.map_q(&q_min, 90).unwrap();
        let mut q_max = [0.0; 5];
        for (i, seg) in robot.segments.iter().enumerate() {
            q_max[i] = seg.joint.value_max;
        }
        let cmd_max = mapper.map_q(&q_max, 90).unwrap();

        for i in 0..5 {
            // q_to_servo zip-trunca al slice más corto: hay que pasar un
            // vector completo y leer el índice i (offsets por joint).
            let mut q_single = [0.0; 5];
            q_single[i] = q_min[i];
            let min_deg = robot.q_to_servo(&q_single)[i].to_degrees();
            q_single[i] = q_max[i];
            let max_deg = robot.q_to_servo(&q_single)[i].to_degrees();
            // Tolerancia flotante: 55°−50° con offsets en radianes da
            // 4.999999999999999, no 5.0 exacto.
            const TOL: f64 = 1e-9;
            assert!(
                (5.0 - TOL..=175.0 + TOL).contains(&min_deg),
                "J{}: q_min={:.2}° → servo {:.2}° FUERA de [5,175] (recorte silencioso)",
                i + 1,
                q_min[i].to_degrees(),
                min_deg
            );
            assert!(
                (5.0 - TOL..=175.0 + TOL).contains(&max_deg),
                "J{}: q_max={:.2}° → servo {:.2}° FUERA de [5,175] (recorte silencioso)",
                i + 1,
                q_max[i].to_degrees(),
                max_deg
            );
            // El comando mapeado no debe desviarse del valor pedido:
            // el clamp no debe haber modificado nada.
            assert!(
                (cmd_min.joints[i] - min_deg).abs() < 1e-6,
                "J{}: mapper modificó q_min ({:.2}° → {:.2}°)",
                i + 1,
                min_deg,
                cmd_min.joints[i]
            );
            assert!(
                (cmd_max.joints[i] - max_deg).abs() < 1e-6,
                "J{}: mapper modificó q_max ({:.2}° → {:.2}°)",
                i + 1,
                max_deg,
                cmd_max.joints[i]
            );
        }
    }

    /// Round-trip q → servo → q con el robot FABRI real (offsets y
    /// direcciones de producción, no el robot de test con todo +1).
    #[test]
    fn fabri_q_servo_round_trip() {
        let robot = crate::robot::fabri_creator();
        let samples: [[f64; 5]; 3] = [
            [0.0, 0.0, 0.0, 0.0, 0.0],
            [0.5, -0.8, 0.3, -0.6, 0.4],
            [-0.7, 0.9, -0.5, 0.8, -0.3],
        ];
        for q in samples.iter() {
            let servo = robot.q_to_servo(q);
            let back = robot.servo_to_q(&servo);
            for i in 0..5 {
                assert!(
                    (back[i] - q[i]).abs() < 1e-9,
                    "J{}: round-trip q={:.6} → servo={:.6} → q={:.6}",
                    i + 1,
                    q[i],
                    servo[i],
                    back[i]
                );
            }
        }
    }
}
