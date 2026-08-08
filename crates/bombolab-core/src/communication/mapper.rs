use super::command::ServoCommand;
use crate::robot::Robot;

pub struct ServoMapper<'a> {
    robot: &'a Robot,
    angle_min: f64,
    angle_max: f64,
}

impl<'a> ServoMapper<'a> {
    pub fn new(robot: &'a Robot) -> Self {
        Self {
            robot,
            angle_min: super::ANGLE_MIN as f64,
            angle_max: super::ANGLE_MAX as f64,
        }
    }

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
        let offset = std::f64::consts::FRAC_PI_2;
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

        let cmd = mapper.map_q(&[-1.5, 0.0, 0.0, 0.0, 0.0], 90).unwrap();
        assert!((cmd.joints[0] - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_clamp_above_max() {
        let robot = make_robot();
        let mapper = ServoMapper::new(&robot);

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

        let cmd = mapper.map_q(&[0.1, -0.2, 0.0, 0.15, -0.1], 45).unwrap();

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

    #[test]
    fn fabri_limits_map_exactly_to_servo_range() {
        let robot = crate::robot::fabri_creator();
        let mapper = ServoMapper::new(&robot);

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
            let mut q_single = [0.0; 5];
            q_single[i] = q_min[i];
            let min_deg = robot.q_to_servo(&q_single)[i].to_degrees();
            q_single[i] = q_max[i];
            let max_deg = robot.q_to_servo(&q_single)[i].to_degrees();

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
