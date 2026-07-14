pub mod hardware;
pub mod ui;

pub use hardware::{MockRobotController, RobotController, SerialRobotController};
pub use ui::main_page::render;
pub use ui::state::AppState;
