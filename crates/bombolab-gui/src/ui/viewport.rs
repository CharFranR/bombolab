// ---------------------------------------------------------------------------
// viewport.rs — Renderizado 3D orbital con egui Painter 2D.
//
// Proyecta puntos 3D a 2D usando rotación de cámara (yaw/pitch) + proyección
// ortográfica. Soporta zoom con scroll, rotación con drag, ejes de coordenadas
// y labels en cada articulación.
// ---------------------------------------------------------------------------

use egui::{Color32, Painter, Pos2, Rect, Stroke};

use crate::ui::state::Camera;

// ---------------------------------------------------------------------------
// Punto 3D
// ---------------------------------------------------------------------------

/// Representa un punto en el espacio 3D del robot.
#[derive(Clone, Copy, Debug, Default)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Point3D {
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    pub const fn origin() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    pub fn magnitude(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
}

// ---------------------------------------------------------------------------
// Proyección orbital
// ---------------------------------------------------------------------------

/// Proyecta un punto 3D a 2D aplicando rotación de cámara (yaw, pitch) seguida
/// de proyección ortográfica.
///
/// La cámara orbita alrededor del origen. Yaw rota alrededor del eje Y,
/// pitch alrededor del eje X (world-space). El resultado es una vista libre
/// del robot desde cualquier ángulo.
fn project_orbital(point: Point3D, cam: &Camera, center: Pos2, scale: f32) -> Pos2 {
    let (sy, cy) = cam.yaw.sin_cos();
    let (sp, cp) = cam.pitch.sin_cos();

    // Ry(yaw): rota alrededor de Y
    let x1 = point.x * cy + point.z * sy;
    let z1 = -point.x * sy + point.z * cy;
    let y1 = point.y;

    // Rx(pitch): rota alrededor de X
    let x2 = x1;
    let z2 = y1 * sp + z1 * cp;

    // Proyección ortográfica: descartamos Y (profundidad), mostramos X y Z
    let screen_x = center.x + x2 * scale * cam.zoom;
    let screen_y = center.y - z2 * scale * cam.zoom;

    Pos2::new(screen_x, screen_y)
}

// ---------------------------------------------------------------------------
// Escala automática con zoom
// ---------------------------------------------------------------------------

/// Calcula la escala base para que el robot ocupe ~40% del viewport.
fn compute_base_scale(points: &[Point3D], viewport_size: f32) -> f32 {
    if viewport_size <= 0.0 {
        return 1.0;
    }
    let max_dist = points
        .iter()
        .map(|p| p.magnitude())
        .fold(0.0f32, f32::max);
    if max_dist < 1e-6 {
        return viewport_size * 0.015;
    }
    viewport_size * 0.40 / max_dist
}

// ---------------------------------------------------------------------------
// Renderizado principal
// ---------------------------------------------------------------------------

/// Renderiza el robot en 3D con cámara orbital.
///
/// Argumentos:
/// - `painter`: painter de egui
/// - `rect`: área del viewport
/// - `points`: puntos 3D del robot (base → joint1 → ... → efector)
/// - `cam`: estado de la cámara (yaw, pitch, zoom)
/// - `joint_color`: color de articulaciones intermedias
/// - `link_color`: color de eslabones
/// - `link_width`: grosor de líneas (px)
/// - `joint_radius`: radio de círculos (px)
/// - `show_labels`: si mostrar etiquetas "J1", "J2", etc.
pub fn draw_robot_skeleton(
    painter: &Painter,
    rect: Rect,
    points: &[Point3D],
    cam: &Camera,
    joint_color: Color32,
    link_color: Color32,
    link_width: f32,
    joint_radius: f32,
    show_labels: bool,
) {
    if points.len() < 2 {
        return;
    }

    let viewport_size = rect.width().min(rect.height());
    let base_scale = compute_base_scale(points, viewport_size);
    let center = rect.center();

    // 1. Plano de suelo
    draw_ground_grid(painter, center, base_scale * cam.zoom, rect, cam);

    // 2. Proyectar puntos
    let projected: Vec<Pos2> = points
        .iter()
        .map(|p| project_orbital(*p, cam, center, base_scale))
        .collect();

    // 3. Eslabones
    for window in projected.windows(2) {
        painter.line_segment([window[0], window[1]], Stroke::new(link_width, link_color));
    }

    // 4. Articulaciones + labels
    let num_points = projected.len();
    let font_id = egui::FontId::proportional(12.0);

    for (i, pos) in projected.iter().enumerate() {
        let color = if i == 0 {
            Color32::from_rgb(160, 160, 160) // base gris
        } else if i == num_points - 1 {
            Color32::from_rgb(80, 220, 80) // efector verde
        } else {
            joint_color
        };

        // Círculo relleno
        painter.circle_filled(*pos, joint_radius, color);
        // Borde blanco sutil
        painter.circle_stroke(
            *pos,
            joint_radius,
            Stroke::new(1.0, Color32::WHITE.gamma_multiply(0.25)),
        );

        // Label
        if show_labels {
            let label = if i == 0 {
                "Base"
            } else if i == num_points - 1 {
                "EE"
            } else {
                // Usar points.len() como proxy: si hay exactamente 6 puntos
                // (base + 5 joints), usar nombres conocidos
                let names = ["J1", "J2", "J3", "J4", "J5"];
                if i - 1 < names.len() {
                    names[i - 1]
                } else {
                    "J"
                }
            };
            let label_pos = Pos2::new(pos.x + joint_radius + 3.0, pos.y - joint_radius - 2.0);
            painter.text(
                label_pos,
                egui::Align2::LEFT_TOP,
                label,
                font_id.clone(),
                Color32::WHITE.gamma_multiply(0.7),
            );
        }
    }
}

// ---------------------------------------------------------------------------
// Plano de suelo
// ---------------------------------------------------------------------------

/// Dibuja un grid en z=0 proyectado con la misma cámara.
fn draw_ground_grid(
    painter: &Painter,
    center: Pos2,
    scale: f32,
    rect: Rect,
    cam: &Camera,
) {
    let grid_size = 3.0;
    let grid_color = Color32::from_rgb(50, 50, 50);
    let step = 1.0;

    // Culling básico: ver si las esquinas proyectadas caen dentro del viewport
    let corners = [
        Point3D::new(-grid_size, -grid_size, 0.0),
        Point3D::new(grid_size, -grid_size, 0.0),
        Point3D::new(grid_size, grid_size, 0.0),
        Point3D::new(-grid_size, grid_size, 0.0),
    ];
    let projected_corners: Vec<Pos2> = corners
        .iter()
        .map(|p| project_orbital(*p, cam, center, scale))
        .collect();
    let grid_bounds = egui::Rect::from_points(&projected_corners);
    if !rect.intersects(grid_bounds) {
        return;
    }

    // Líneas paralelas a X (en z=0)
    let mut y = -grid_size;
    while y <= grid_size {
        let p1 = project_orbital(Point3D::new(-grid_size, y, 0.0), cam, center, scale);
        let p2 = project_orbital(Point3D::new(grid_size, y, 0.0), cam, center, scale);
        painter.line_segment([p1, p2], Stroke::new(0.5, grid_color));
        y += step;
    }

    // Líneas paralelas a Y (en z=0)
    let mut x = -grid_size;
    while x <= grid_size {
        let p1 = project_orbital(Point3D::new(x, -grid_size, 0.0), cam, center, scale);
        let p2 = project_orbital(Point3D::new(x, grid_size, 0.0), cam, center, scale);
        painter.line_segment([p1, p2], Stroke::new(0.5, grid_color));
        x += step;
    }
}

// ---------------------------------------------------------------------------
// Ejes de coordenadas (esquina inferior izquierda del viewport)
// ---------------------------------------------------------------------------

/// Dibuja los ejes X (rojo), Y (verde), Z (azul) en una esquina.
///
/// La longitud de cada eje es `size` mm en el espacio del robot.
pub fn draw_axes(painter: &Painter, rect: Rect, cam: &Camera, scale: f32, size: f32) {
    let origin_screen = Pos2::new(rect.left() + 40.0, rect.bottom() - 40.0);

    let axes = [
        ("X", Point3D::new(size, 0.0, 0.0), Color32::RED),
        ("Y", Point3D::new(0.0, size, 0.0), Color32::GREEN),
        ("Z", Point3D::new(0.0, 0.0, size), Color32::BLUE),
    ];

    let font_id = egui::FontId::proportional(11.0);

    for (label, tip, color) in axes {
        let tip_screen = project_orbital(tip, cam, origin_screen, scale);
        let origin_here = project_orbital(Point3D::origin(), cam, origin_screen, scale);
        painter.line_segment([origin_here, tip_screen], Stroke::new(2.0, color));
        painter.text(tip_screen, egui::Align2::CENTER_CENTER, label, font_id.clone(), color);
    }
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_point3d_origin() {
        let p = Point3D::origin();
        assert_eq!(p.x, 0.0);
        assert_eq!(p.y, 0.0);
        assert_eq!(p.z, 0.0);
        assert!(p.magnitude() < 1e-6);
    }

    #[test]
    fn test_point3d_magnitude() {
        let p = Point3D::new(3.0, 4.0, 0.0);
        assert!((p.magnitude() - 5.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_base_scale_default() {
        let points = vec![Point3D::origin(), Point3D::origin()];
        let scale = compute_base_scale(&points, 500.0);
        assert!((scale - 7.5).abs() < 1e-6);
    }

    #[test]
    fn test_compute_base_scale_dynamic() {
        let points = vec![Point3D::origin(), Point3D::new(10.0, 0.0, 0.0)];
        let scale = compute_base_scale(&points, 500.0);
        assert!((scale - 20.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_base_scale_zero_viewport() {
        let points = vec![Point3D::origin(), Point3D::new(1.0, 0.0, 0.0)];
        let scale = compute_base_scale(&points, 0.0);
        assert!((scale - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_project_orbital_origin() {
        let cam = Camera::new();
        let center = Pos2::new(100.0, 100.0);
        let projected = project_orbital(Point3D::origin(), &cam, center, 1.0);
        assert!((projected.x - 100.0).abs() < 1e-6);
        assert!((projected.y - 100.0).abs() < 1e-6);
    }
}
