use std::fmt;

#[derive(PartialEq)]
pub enum JointType {
    Revolute,
    Prismatic,
}

impl fmt::Display for JointType {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            JointType::Revolute => write!(f, "R"),
            JointType::Prismatic => write!(f, "P"),
        }
    }
}

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
