use wasm_bindgen::prelude::*;

use bombolab_core::kinematics::{
    IkSolver, OrientationSolver, forward_kinematics as fk, solve_drawing_ik as solve_drawing,
    solve_drawing_ik_v2 as solve_drawing_v2,
};
use bombolab_core::math::Iso3;
use bombolab_core::robot::{
    Joint, JointType, Robot, Segment, base_transform as make_base_transform,
    fabri_creator as make_fabri_creator, tool_transform as make_tool_transform,
};

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
        m[(0, 0)],
        m[(0, 1)],
        m[(0, 2)],
        m[(0, 3)],
        m[(1, 0)],
        m[(1, 1)],
        m[(1, 2)],
        m[(1, 3)],
        m[(2, 0)],
        m[(2, 1)],
        m[(2, 2)],
        m[(2, 3)],
    ]
}

fn array_to_iso3(arr: &[f64; 12]) -> Iso3 {
    use nalgebra::{Translation3, UnitQuaternion};
    let translation = Translation3::new(arr[3], arr[7], arr[11]);
    // Extract rotation columns from 3x4 matrix (ignoring translation)
    let r00 = arr[0];
    let r01 = arr[1];
    let r02 = arr[2];
    let r10 = arr[4];
    let r11 = arr[5];
    let r12 = arr[6];
    let r20 = arr[8];
    let r21 = arr[9];
    let r22 = arr[10];
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
    let segments = js_robot
        .segments
        .iter()
        .map(|s| {
            let joint = bombolab_core::robot::Joint::new(
                joint_type_from_str(&s.joint_type),
                s.q,
                s.q_max,
                s.q_min,
            );
            let dh = bombolab_core::robot::DHParams::new(s.theta, s.d, s.a, s.alpha);
            bombolab_core::robot::Segment::new(joint, dh)
        })
        .collect();
    Robot::new(segments)
}

// ─── Public API ─────────────────────────────────────────────────────────────

/// Helper para serializar con `serde_wasm_bindgen`, mapeando el error a JS.
fn to_js_value<T: serde::Serialize>(value: &T) -> Result<JsValue, JsValue> {
    serde_wasm_bindgen::to_value(value).map_err(|e| JsValue::from_str(&e.to_string()))
}

/// Deserializa un `JsRobotDef` desde JS, mapeando el error a JS.
fn robot_from_js_value(js_robot: &JsValue) -> Result<JsRobotDef, JsValue> {
    serde_wasm_bindgen::from_value(js_robot.clone()).map_err(|e| JsValue::from_str(&e.to_string()))
}

/// Return the FABRI Creator robot definition.
///
/// Returns a `Result`; on error a JS exception is thrown (never traps).
#[wasm_bindgen]
pub fn fabri_creator() -> Result<JsValue, JsValue> {
    let robot = make_fabri_creator();
    let base = make_base_transform();
    let tool = make_tool_transform();

    let js_robot = JsRobotDef {
        segments: robot
            .segments
            .iter()
            .map(|seg| JsSegment {
                q: seg.joint.value,
                theta: seg.dh.theta,
                d: seg.dh.d,
                a: seg.dh.a,
                alpha: seg.dh.alpha,
                q_min: seg.joint.value_min,
                q_max: seg.joint.value_max,
                joint_type: joint_type_to_str(&seg.joint.joint_type).to_string(),
            })
            .collect(),
        base_transform: iso3_to_array(&base),
        tool_transform: iso3_to_array(&tool),
    };

    to_js_value(&js_robot)
}

/// Forward kinematics: compute all frames for given q.
///
/// Returns a `Result`; on malformed input a JS exception is thrown
/// (never traps).
#[wasm_bindgen]
pub fn forward_kinematics(js_robot: &JsValue) -> Result<JsValue, JsValue> {
    let js_robot = robot_from_js_value(js_robot)?;
    let robot = robot_from_js(&js_robot);
    let base = array_to_iso3(&js_robot.base_transform);

    let (frames, _ee) = fk(base, &robot);

    let tool = array_to_iso3(&js_robot.tool_transform);
    let tool_pose = match frames.last() {
        Some(frame) => *frame * tool,
        None => return Err(JsValue::from_str("robot has no segments")),
    };

    let result = JsFkResult {
        frames: frames.iter().map(iso3_to_array).collect(),
        ee: iso3_to_array(&tool_pose),
    };

    to_js_value(&result)
}

/// Inverse kinematics: solve position [x, y, z] → q.
///
/// Returns a `Result`; on malformed input a JS exception is thrown
/// (never traps). Unreachable targets are reported as `converged: false`.
#[wasm_bindgen]
pub fn solve_ik(js_robot: &JsValue, target: &[f64], q_init: &[f64]) -> Result<JsValue, JsValue> {
    let js_robot = robot_from_js_value(js_robot)?;
    let robot = robot_from_js(&js_robot);
    let base = array_to_iso3(&js_robot.base_transform);
    let tool = array_to_iso3(&js_robot.tool_transform);

    if target.len() < 3 {
        return Err(JsValue::from_str("target must have at least 3 values"));
    }
    let target_arr = [target[0], target[1], target[2]];

    let solver = IkSolver::new(200, 1.0, 0.05, 0.5);

    match solver.solve_position(&target_arr, q_init, &robot, &base, &tool) {
        Ok(q) => {
            // Re-compute FK with solved q to get actual error
            let solved_robot = {
                let segments: Vec<_> = robot
                    .segments
                    .iter()
                    .zip(q.iter())
                    .map(|(seg, &val)| {
                        let joint = Joint::new(
                            seg.joint.joint_type,
                            val,
                            seg.joint.value_max,
                            seg.joint.value_min,
                        );
                        Segment::new(joint, seg.dh) // DHParams es Copy
                    })
                    .collect();
                Robot::new(segments)
            };
            let (frames, _) = fk(base, &solved_robot);
            let tool_pose = frames
                .last()
                .ok_or_else(|| JsValue::from_str("robot has no segments"))?
                * tool;
            let p_ee = tool_pose.translation.vector;
            let target_v = nalgebra::Vector3::new(target_arr[0], target_arr[1], target_arr[2]);
            let error = (target_v - p_ee).norm();

            let result = JsIkResult {
                q,
                converged: true,
                error,
            };
            to_js_value(&result)
        }
        Err(e) => match e {
            // Inalcanzable: respuesta normal con converged=false.
            bombolab_core::kinematics::IkError::MaxIterationsReached { error } => {
                let result = JsIkResult {
                    q: q_init.to_vec(),
                    converged: false,
                    error,
                };
                to_js_value(&result)
            }
            // Error de entrada del caller: lanzar excepción.
            other => Err(JsValue::from_str(&other.to_string())),
        },
    }
}

/// Inverse kinematics with drawing mode: solve position + orientation for drawing.
///
/// Uses `solve_drawing_ik` which generates an adaptive orientation target
/// (R_target with θ = q₁ + π) so the marker stays perpendicular to the XY plane.
///
/// Si la orientación no es alcanzable, cae suavemente a solo posición
/// (el robot se mueve al target aunque el marcador no esté perfectamente vertical).
///
/// Returns a `Result`; on malformed input a JS exception is thrown
/// (never traps).
#[wasm_bindgen]
pub fn solve_drawing_ik(
    js_robot: &JsValue,
    target: &[f64],
    q_init: &[f64],
) -> Result<JsValue, JsValue> {
    let js_robot = robot_from_js_value(js_robot)?;
    let robot = robot_from_js(&js_robot);
    let base = array_to_iso3(&js_robot.base_transform);
    let tool = array_to_iso3(&js_robot.tool_transform);

    if target.len() < 3 {
        return Err(JsValue::from_str("target must have at least 3 values"));
    }
    let target_arr = [target[0], target[1], target[2]];

    let pos_solver = IkSolver::new(200, 1.0, 0.05, 0.5);
    let orient_solver = OrientationSolver::new(1e-6);

    // 1. Intentar IK completa con orientación adaptativa
    match solve_drawing(
        &pos_solver,
        &orient_solver,
        &target_arr,
        q_init,
        &robot,
        &base,
        &tool,
    ) {
        Ok(q) => {
            // Éxito: devolver solución completa
            let error = compute_position_error(&robot, &q, &target_arr, &base, &tool)?;
            let result = JsIkResult {
                q,
                converged: true,
                error,
            };
            to_js_value(&result)
        }
        Err(_) => {
            // 2. Fallback: solo posición (sin orientación)
            match pos_solver.solve_position(&target_arr, q_init, &robot, &base, &tool) {
                Ok(q) => {
                    let error = compute_position_error(&robot, &q, &target_arr, &base, &tool)?;
                    let result = JsIkResult {
                        q,
                        converged: true,
                        error,
                    };
                    to_js_value(&result)
                }
                Err(e) => match e {
                    bombolab_core::kinematics::IkError::MaxIterationsReached { error } => {
                        let result = JsIkResult {
                            q: q_init.to_vec(),
                            converged: false,
                            error,
                        };
                        to_js_value(&result)
                    }
                    other => Err(JsValue::from_str(&other.to_string())),
                },
            }
        }
    }
}

/// Inverse kinematics drawing mode 2: marker along Y₅ axis.
///
/// El marcador está montado perpendicular al gripper, apuntando en Y₅.
/// Usa `drawing_pose_v2` con R_target que mantiene Y₅ = -Z (vertical).
///
/// Fallback a solo posición si la orientación no es alcanzable.
///
/// Returns a `Result`; on malformed input a JS exception is thrown
/// (never traps).
#[wasm_bindgen]
pub fn solve_drawing_ik_v2(
    js_robot: &JsValue,
    target: &[f64],
    q_init: &[f64],
) -> Result<JsValue, JsValue> {
    let js_robot = robot_from_js_value(js_robot)?;
    let robot = robot_from_js(&js_robot);
    let base = array_to_iso3(&js_robot.base_transform);
    let tool = array_to_iso3(&js_robot.tool_transform);

    if target.len() < 3 {
        return Err(JsValue::from_str("target must have at least 3 values"));
    }
    let target_arr = [target[0], target[1], target[2]];

    let pos_solver = IkSolver::new(200, 1.0, 0.05, 0.5);
    let orient_solver = OrientationSolver::new(1e-6);

    match solve_drawing_v2(
        &pos_solver,
        &orient_solver,
        &target_arr,
        q_init,
        &robot,
        &base,
        &tool,
    ) {
        Ok(q) => {
            let error = compute_position_error(&robot, &q, &target_arr, &base, &tool)?;
            let result = JsIkResult {
                q,
                converged: true,
                error,
            };
            to_js_value(&result)
        }
        Err(_) => match pos_solver.solve_position(&target_arr, q_init, &robot, &base, &tool) {
            Ok(q) => {
                let error = compute_position_error(&robot, &q, &target_arr, &base, &tool)?;
                let result = JsIkResult {
                    q,
                    converged: true,
                    error,
                };
                to_js_value(&result)
            }
            Err(e) => match e {
                bombolab_core::kinematics::IkError::MaxIterationsReached { error } => {
                    let result = JsIkResult {
                        q: q_init.to_vec(),
                        converged: false,
                        error,
                    };
                    to_js_value(&result)
                }
                other => Err(JsValue::from_str(&other.to_string())),
            },
        },
    }
}

/// Inverse kinematics for the drawing-plane mode (marker vertical).
///
/// Solves directly inside the constrained manifold
/// `M = { q : q4 = 0, q5 = −(q2+q3) }` with the chain-rule reduced
/// Jacobian `[J₁, J₂−J₅, J₃−J₅]` — the wrist is never free, so every
/// iterate is a valid drawing pose and the TCP lands on target with the
/// marker vertical (no post-hoc q4/q5 correction, no bad wrist branches).
///
/// Targets outside the drawing workspace (J5 pitch limit on q5 = −q23)
/// return `converged: false` — a normal non-convergence, not an exception.
#[wasm_bindgen]
pub fn solve_drawing_plane_ik(
    js_robot: &JsValue,
    target: &[f64],
    q_init: &[f64],
) -> Result<JsValue, JsValue> {
    let js_robot = robot_from_js_value(js_robot)?;
    let robot = robot_from_js(&js_robot);
    let base = array_to_iso3(&js_robot.base_transform);
    let tool = array_to_iso3(&js_robot.tool_transform);

    if target.len() < 3 {
        return Err(JsValue::from_str("target must have at least 3 values"));
    }
    let target_arr = [target[0], target[1], target[2]];

    let solver = IkSolver::new(200, 1.0, 0.05, 0.5);

    match bombolab_core::kinematics::solve_drawing_plane_ik(
        &solver,
        &target_arr,
        q_init,
        &robot,
        &base,
        &tool,
    ) {
        Ok(q) => {
            let error = compute_position_error(&robot, &q, &target_arr, &base, &tool)?;
            let result = JsIkResult {
                q: q.to_vec(),
                converged: true,
                error,
            };
            to_js_value(&result)
        }
        Err(e) => match e {
            bombolab_core::kinematics::IkError::MaxIterationsReached { .. }
            | bombolab_core::kinematics::IkError::DrawingConstraintViolated { .. } => {
                // Out of the drawing workspace or no convergence: report the
                // current pose's error as a normal non-convergence.
                let error = compute_position_error(&robot, q_init, &target_arr, &base, &tool)?;
                let result = JsIkResult {
                    q: q_init.to_vec(),
                    converged: false,
                    error,
                };
                to_js_value(&result)
            }
            other => Err(JsValue::from_str(&other.to_string())),
        },
    }
}

/// Helper: compute FK position error for solved q values.
///
/// Returns a `Result` so an empty robot chain surfaces as a JS error
/// instead of trapping on `frames.last().unwrap()`.
fn compute_position_error(
    robot: &Robot,
    q: &[f64],
    target: &[f64; 3],
    base: &Iso3,
    tool: &Iso3,
) -> Result<f64, JsValue> {
    use bombolab_core::robot::{Joint, Robot, Segment};
    let solved = {
        let segments: Vec<_> = robot
            .segments
            .iter()
            .zip(q.iter())
            .map(|(seg, &val)| {
                let joint = Joint::new(
                    seg.joint.joint_type,
                    val,
                    seg.joint.value_max,
                    seg.joint.value_min,
                );
                Segment::new(joint, seg.dh) // DHParams es Copy
            })
            .collect();
        Robot::new(segments)
    };
    let (frames, _) = fk(*base, &solved);
    let tool_pose = frames
        .last()
        .ok_or_else(|| JsValue::from_str("robot has no segments"))?
        * tool;
    let p_ee = tool_pose.translation.vector;
    let target_v = nalgebra::Vector3::new(target[0], target[1], target[2]);
    Ok((target_v - p_ee).norm())
}
