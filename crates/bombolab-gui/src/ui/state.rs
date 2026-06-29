use bombolab_core::{DHParams, Joint, JointType, Robot, Segment};

pub struct SegmentUi {
    pub joint_type: JointType,
    pub theta: f64,
    pub d: f64,
    pub a: f64,
    pub alpha: f64,
}

impl SegmentUi {
    pub fn new_revolute() -> Self {
        Self {
            joint_type: JointType::Revolute,
            theta: 0.0,
            d: 0.0,
            a: 1.0,
            alpha: 0.0,
        }
    }

    pub fn to_segment(&self, joint_value: f64) -> Segment {
        let joint = Joint::new(self.joint_type, joint_value, 0.0, 0.0);
        let dh = DHParams::new(self.theta, self.d, self.a, self.alpha);
        Segment::new(joint, dh)
    }
}

pub struct RobotDef {
    pub name: String,
    pub segments: Vec<SegmentUi>,
}

impl RobotDef {
    pub fn new(name: &str) -> Self {
        Self {
            name: name.to_string(),
            segments: Vec::new(),
        }
    }

    pub fn dof(&self) -> usize {
        self.segments.len()
    }

    pub fn to_robot(&self) -> Robot {
        let segments: Vec<Segment> = self.segments.iter().map(|s| s.to_segment(0.0)).collect();
        Robot::new(segments)
    }
}

#[derive(PartialEq)]
pub enum PanelView {
    Main,
    RobotList,
    RobotEditor(usize),
    Movements,
}

pub struct AppState {
    pub view: PanelView,
    pub robots: Vec<RobotDef>,
    pub selected_robot: Option<usize>,
    pub show_details: bool,
}

impl AppState {
    pub fn new() -> Self {
        Self {
            view: PanelView::Main,
            robots: Vec::new(),
            selected_robot: None,
            show_details: false,
        }
    }
}
