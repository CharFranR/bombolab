use wasm_bindgen::prelude::*;

use bombolab_core::math::Iso3;
use bombolab_core::robot::{fabri_creator as make_fabri_creator, base_transform as make_base_transform, tool_transform as make_tool_transform, DHParams, Joint, JointType, Robot, Segment};
use bombolab_core::kinematics::{forward_kinematics as fk, IkSolver};

// ─── Serializable types for JS interop ──────────────────────────────────────

#[derive(serde::Serialize, serde::Deserialize)]
pub struct JsSegment {
    pub q: f64,
    pub theta: f64,
    pub d: f64,
    pub a: f64,
    pub alpha: f64,
    pub q_min: f64,
    pub q_max: f64,
    pub joint_type: String,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct JsRobotDef {
    pub segments: Vec<JsSegment>,
    pub base_transform: [f64; 12],
    pub tool_transform: [f64; 12],
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct JsIkResult {
    pub q: Vec<f64>,
    pub converged: bool,
    pub error: f64,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct JsFkResult {
    pub frames: Vec<[f64; 12]>,
    pub ee: [f64; 12],
}

// ─── Helpers ────────────────────────────────────────────────────────────────

fn iso3_to_array(t: &Iso3) -> [f64; 12] {
    let m = t.to_matrix();
    [
        m[(0, 0)], m[(0, 1)], m[(0, 2)], m[(0, 3)],
        m[(1, 0)], m[(1, 1)], m[(1, 2)], m[(1, 3)],
        m[(2, 0)], m[(2, 1)], m[(2, 2)], m[(2, 3)],
    ]
}

fn array_to_iso3(arr: &[f64; 12]) -> Iso3 {
    use nalgebra::{Translation3, UnitQuaternion};
    let translation = Translation3::new(arr[3], arr[7], arr[11]);
    // Extract rotation columns from 3x4 matrix (ignoring translation)
    let r00 = arr[0]; let r01 = arr[1]; let r02 = arr[2];
    let r10 = arr[4]; let r11 = arr[5]; let r12 = arr[6];
    let r20 = arr[8]; let r21 = arr[9]; let r22 = arr[10];
    // Build rotation matrix and convert to quaternion
    let rot = nalgebra::Matrix3::new(r00, r01, r02, r10, r11, r12, r20, r21, r22);
    let rotation = UnitQuaternion::from_matrix(&rot);
    Iso3::from_parts(translation, rotation)
}

fn joint_type_to_str(jt: &JointType) -> &'static str {
    match jt {
        JointType::Revolute => "revolute",
        JointType::Prismatic => "prismatic",
        JointType::Twist => "twist",
    }
}

fn joint_type_from_str(s: &str) -> JointType {
    match s {
        "revolute" => JointType::Revolute,
        "prismatic" => JointType::Prismatic,
        "twist" => JointType::Twist,
        _ => JointType::Revolute,
    }
}

fn robot_from_js(js_robot: &JsRobotDef) -> Robot {
    let segments = js_robot.segments.iter().map(|s| {
        let joint = bombolab_core::robot::Joint::new(
            joint_type_from_str(&s.joint_type),
            s.q,
            s.q_max,
            s.q_min,
        );
        let dh = bombolab_core::robot::DHParams::new(s.theta, s.d, s.a, s.alpha);
        bombolab_core::robot::Segment::new(joint, dh)
    }).collect();

    Robot::new(segments)
}

// ─── Public API ─────────────────────────────────────────────────────────────

/// Return the FABRI Creator robot definition.
#[wasm_bindgen]
pub fn fabri_creator() -> JsValue {
    let robot = make_fabri_creator();
    let base = make_base_transform();
    let tool = make_tool_transform();

    let js_robot = JsRobotDef {
        segments: robot.segments.iter().map(|seg| {
            JsSegment {
                q: seg.joint.value,
                theta: seg.dh.theta,
                d: seg.dh.d,
                a: seg.dh.a,
                alpha: seg.dh.alpha,
                q_min: seg.joint.value_min,
                q_max: seg.joint.value_max,
                joint_type: joint_type_to_str(&seg.joint.joint_type).to_string(),
            }
        }).collect(),
        base_transform: iso3_to_array(&base),
        tool_transform: iso3_to_array(&tool),
    };

    serde_wasm_bindgen::to_value(&js_robot).unwrap()
}

/// Forward kinematics: compute all frames for given q.
#[wasm_bindgen]
pub fn forward_kinematics(js_robot: &JsValue) -> JsValue {
    let js_robot: JsRobotDef = serde_wasm_bindgen::from_value(js_robot.clone()).unwrap();
    let robot = robot_from_js(&js_robot);
    let base = array_to_iso3(&js_robot.base_transform);

    let (frames, _ee) = fk(base, &robot);

    let tool = array_to_iso3(&js_robot.tool_transform);
    let tool_pose = *frames.last().unwrap() * tool;

    let result = JsFkResult {
        frames: frames.iter().map(iso3_to_array).collect(),
        ee: iso3_to_array(&tool_pose),
    };

    serde_wasm_bindgen::to_value(&result).unwrap()
}

/// Inverse kinematics: solve position [x, y, z] → q.
#[wasm_bindgen]
pub fn solve_ik(js_robot: &JsValue, target: &[f64], q_init: &[f64]) -> JsValue {
    let js_robot: JsRobotDef = serde_wasm_bindgen::from_value(js_robot.clone()).unwrap();
    let robot = robot_from_js(&js_robot);
    let base = array_to_iso3(&js_robot.base_transform);
    let tool = array_to_iso3(&js_robot.tool_transform);

    let solver = IkSolver::new(200, 1.0, 0.05, 0.5);
    let target_arr = [target[0], target[1], target[2]];

    match solver.solve_position(&target_arr, q_init, &robot, &base, &tool) {
        Ok(q) => {
            // Re-compute FK with solved q to get actual error
            let solved_robot = {
                let segments: Vec<_> = robot.segments.iter().zip(q.iter()).map(|(seg, &val)| {
                    let joint = Joint::new(seg.joint.joint_type, val, seg.joint.value_max, seg.joint.value_min);
                    Segment::new(joint, DHParams::new(seg.dh.theta, seg.dh.d, seg.dh.a, seg.dh.alpha))
                }).collect();
                Robot::new(segments)
            };
            let (frames, _) = fk(base, &solved_robot);
            let tool_pose = frames.last().unwrap() * tool;
            let p_ee = tool_pose.translation.vector;
            let target_v = nalgebra::Vector3::new(target_arr[0], target_arr[1], target_arr[2]);
            let error = (target_v - p_ee).norm();

            let result = JsIkResult { q, converged: true, error };
            serde_wasm_bindgen::to_value(&result).unwrap()
        }
        Err(e) => {
            let result = JsIkResult {
                q: q_init.to_vec(),
                converged: false,
                error: match e {
                    bombolab_core::kinematics::IkError::MaxIterationsReached { error } => error,
                    _ => f64::MAX,
                },
            };
            serde_wasm_bindgen::to_value(&result).unwrap()
        }
    }
}

/// Get base transform as 4x3 matrix (row-major, 12 floats).
#[wasm_bindgen]
pub fn base_transform() -> JsValue {
    let t = make_base_transform();
    let arr = iso3_to_array(&t);
    serde_wasm_bindgen::to_value(&arr).unwrap()
}

/// Get tool transform as 4x3 matrix (row-major, 12 floats).
#[wasm_bindgen]
pub fn tool_transform() -> JsValue {
    let t = make_tool_transform();
    let arr = iso3_to_array(&t);
    serde_wasm_bindgen::to_value(&arr).unwrap()
}
