mod domain;
mod kinematics;
mod ui;

use ui::state::AppState;

struct App {
    state: AppState,
}

impl eframe::App for App {
    fn ui(&mut self, ui: &mut egui::Ui, _frame: &mut eframe::Frame) {
        ui::main_page::render(ui, &mut self.state);
    }
}

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1000.0, 650.0])
            .with_title("Bombolab"),
        ..Default::default()
    };

    eframe::run_native("Bombolab", options, Box::new(|_cc| {
        Ok(Box::new(App {
            state: AppState::new(),
        }))
    }))
}
