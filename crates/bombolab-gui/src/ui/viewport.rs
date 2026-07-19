// ---------------------------------------------------------------------------
// viewport.rs — Renderizado 3D orbital con egui Painter 2D.
//
// Proyecta puntos 3D a 2D usando rotación de cámara (yaw/pitch) + proyección
// ortográfica. Soporta zoom con scroll, rotación con drag, ejes de coordenadas
// y renderizado de links como cuerpos 3D con volumen.
// ---------------------------------------------------------------------------

use egui::{Color32, Painter, Pos2, Rect, Shape, Stroke};

use crate::ui::state::Camera;

// ---------------------------------------------------------------------------
// Constantes de cuerpo del robot (mm)
// ---------------------------------------------------------------------------

/// Ancho físico de los eslabones del brazo (simula el cuerpo del robot).
const LINK_WIDTH_MM: f32 = 18.0;
/// Ancho de la base del robot.
const BASE_WIDTH_MM: f32 = 50.0;
/// Alto de la base del robot.
const BASE_HEIGHT_MM: f32 = 15.0;
/// Tamaño del grid de piso
const GRID_SIZE: f32 = 12.0;
/// Separación entre líneas del grid
const GRID_STEP: f32 = 2.0;

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
fn project_orbital(point: Point3D, cam: &Camera, center: Pos2, scale: f32) -> Pos2 {
    let (sy, cy) = cam.yaw.sin_cos();
    let (sp, cp) = cam.pitch.sin_cos();

    // Ry(yaw)
    let x1 = point.x * cy + point.z * sy;
    let z1 = -point.x * sy + point.z * cy;

    // Rx(pitch)
    let x2 = x1;
    let z2 = point.y * sp + z1 * cp;

    let screen_x = center.x + x2 * scale * cam.zoom;
    let screen_y = center.y - z2 * scale * cam.zoom;

    Pos2::new(screen_x, screen_y)
}

/// Versión con 3 componentes (x, y, z) devuelve las 2D proyectadas + la
/// profundidad Y (para ordenar dibujo back-to-front).
fn project_orbital_depth(
    point: Point3D,
    cam: &Camera,
    center: Pos2,
    scale: f32,
) -> (Pos2, f32) {
    let (sy, cy) = cam.yaw.sin_cos();
    let (sp, cp) = cam.pitch.sin_cos();

    let x1 = point.x * cy + point.z * sy;
    let y1 = point.y;
    let z1 = -point.x * sy + point.z * cy;

    let x2 = x1;
    let y2 = y1 * cp - z1 * sp;
    let z2 = y1 * sp + z1 * cp;

    let screen_x = center.x + x2 * scale * cam.zoom;
    let screen_y = center.y - z2 * scale * cam.zoom;

    (Pos2::new(screen_x, screen_y), y2) // y2 = profundidad (back-to-front)
}

// ---------------------------------------------------------------------------
// Escala
// ---------------------------------------------------------------------------

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
// Primitivas de dibujo 3D → 2D
// ---------------------------------------------------------------------------

/// Dibuja un eslabón como un cuerpo rectangular con volumen.
///
/// En lugar de una línea fina, pinta un rectángulo relleno entre `from` y `to`
/// con el ancho dado. El color se aclara/oscurece según la profundidad para
/// dar sensación 3D.
fn draw_link_body(
    painter: &Painter,
    from: Pos2,
    to: Pos2,
    width_screen: f32,
    color: Color32,
) {
    let dir = (to - from).normalized();
    let perp = egui::vec2(-dir.y, dir.x);
    let half = width_screen * 0.5;

    let p1 = from + perp * half;
    let p2 = from - perp * half;
    let p3 = to - perp * half;
    let p4 = to + perp * half;

    painter.add(Shape::convex_polygon(
        vec![p1, p2, p3, p4],
        color,
        Stroke::new(1.0, color.gamma_multiply(0.6)),
    ));
}

/// Dibuja una articulación como un círculo con brillo y borde.
fn draw_joint(painter: &Painter, pos: Pos2, radius: f32, color: Color32) {
    // Sombra / halo exterior
    painter.circle_filled(pos, radius + 2.0, color.gamma_multiply(0.2));
    // Relleno
    painter.circle_filled(pos, radius, color);
    // Brillo superior-izquierdo (simula luz)
    painter.circle_filled(
        Pos2::new(pos.x - radius * 0.25, pos.y - radius * 0.25),
        radius * 0.35,
        Color32::WHITE.gamma_multiply(0.2),
    );
}

// ---------------------------------------------------------------------------
// Renderizado principal
// ---------------------------------------------------------------------------

/// Renderiza el robot con cuerpos 3D (no palitos).
pub fn draw_robot_skeleton(
    painter: &Painter,
    rect: Rect,
    points: &[Point3D],
    cam: &Camera,
    joint_color: Color32,
    link_color: Color32,
    _link_width: f32,
    joint_radius: f32,
    show_labels: bool,
) {
    if points.len() < 2 {
        return;
    }

    let viewport_size = rect.width().min(rect.height());
    let base_scale = compute_base_scale(points, viewport_size);
    let eff_scale = base_scale * cam.zoom;
    let center = rect.center();

    // 1. Plano de suelo (más grande y vistoso)
    draw_ground_plane(painter, center, eff_scale, rect, cam);

    // 2. Proyectar puntos con profundidad para z-sorting
    let projected: Vec<(Pos2, f32)> = points
        .iter()
        .map(|p| project_orbital_depth(*p, cam, center, base_scale))
        .collect();

    // 3. Z-sort: dibujar links de atrás hacia adelante
    //    Calculamos profundidad media de cada link
    struct Link {
        depth: f32,
        from: Pos2,
        to: Pos2,
    }

    let mut links: Vec<Link> = projected
        .windows(2)
        .map(|w| Link {
            depth: (w[0].1 + w[1].1) * 0.5,
            from: w[0].0,
            to: w[1].0,
        })
        .collect();

    links.sort_by(|a, b| a.depth.partial_cmp(&b.depth).unwrap());

    // 4. Dibujar links como cuerpos (de atrás → adelante)
    let link_radius_mm = LINK_WIDTH_MM;
    let link_width_px = link_radius_mm * eff_scale;

    for link in &links {
        // Color con variación de profundidad para efecto 3D
        let depth_factor = (link.depth / 500.0).clamp(-0.3, 0.3);
        let r = (link_color.r() as f32 * (1.0 + depth_factor)).clamp(0.0, 255.0) as u8;
        let g = (link_color.g() as f32 * (1.0 + depth_factor)).clamp(0.0, 255.0) as u8;
        let b = (link_color.b() as f32 * (1.0 + depth_factor)).clamp(0.0, 255.0) as u8;
        let body_color = Color32::from_rgb(r, g, b);

        draw_link_body(painter, link.from, link.to, link_width_px, body_color);
    }

    // 5. Base sólida (un cubo achatado en el ground)
    let ground_pos = projected[0].0;
    let base_w = BASE_WIDTH_MM * eff_scale;
    let base_h = BASE_HEIGHT_MM * eff_scale;
    let base_color = Color32::from_rgb(100, 100, 110);

    // Dibujar base como rectángulo centrado en el ground
    let base_rect = egui::Rect::from_center_size(ground_pos, egui::vec2(base_w, base_h * 0.6));
    painter.add(Shape::rect_filled(base_rect, 2.0, base_color));
    painter.add(Shape::rect_stroke(
        base_rect,
        2.0,
        Stroke::new(1.0, Color32::from_rgb(140, 140, 150)),
        egui::StrokeKind::Inside,
    ));

    // 6. Articulaciones (círculos con brillo) — dibujar de atrás hacia adelante
    let mut joint_infos: Vec<(f32, usize, Pos2)> = projected
        .iter()
        .enumerate()
        .map(|(i, (pos, depth))| (*depth, i, *pos))
        .collect();
    joint_infos.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());

    let num_points = points.len();
    for (_depth, i, pos) in &joint_infos {
        let color = if *i == 0 {
            Color32::from_rgb(130, 130, 140) // base
        } else if *i == num_points - 1 {
            Color32::from_rgb(70, 210, 70) // efector
        } else {
            joint_color
        };

        let radius = if *i == 0 || *i == num_points - 1 {
            joint_radius * 1.3
        } else {
            joint_radius
        };

        draw_joint(painter, *pos, radius, color);
    }

    // 7. Labels
    if show_labels {
        let font_id = egui::FontId::proportional(12.0);
        let label_names: Vec<&str> = {
            let mut names = vec!["Base"];
            for i in 1..num_points.saturating_sub(1) {
                let n = match i {
                    1 => "J1",
                    2 => "J2",
                    3 => "J3",
                    4 => "J4",
                    5 => "J5",
                    _ => "",
                };
                names.push(n);
            }
            if num_points > 1 {
                names.push("EE");
            }
            names
        };

        for (i, (pos, _depth)) in projected.iter().enumerate() {
            if let Some(label) = label_names.get(i) {
                let label_pos = Pos2::new(pos.x + joint_radius + 4.0, pos.y - joint_radius - 3.0);
                painter.text(
                    label_pos,
                    egui::Align2::LEFT_TOP,
                    *label,
                    font_id.clone(),
                    Color32::WHITE.gamma_multiply(0.8),
                );
            }
        }
    }
}

// ---------------------------------------------------------------------------
// Plano de suelo
// ---------------------------------------------------------------------------

/// Dibuja un plano de suelo grande con relleno semitransparente y líneas de grid.
fn draw_ground_plane(painter: &Painter, center: Pos2, scale: f32, rect: Rect, cam: &Camera) {
    // Relleno semitransparente del piso (solo si hay suficientes puntos proyectados)
    let s = GRID_SIZE;
    let corners_3d = [
        Point3D::new(-s, -s, 0.0),
        Point3D::new(s, -s, 0.0),
        Point3D::new(s, s, 0.0),
        Point3D::new(-s, s, 0.0),
    ];
    let projected_corners: Vec<Pos2> = corners_3d
        .iter()
        .map(|p| project_orbital(*p, cam, center, scale))
        .collect();
    let grid_bounds = egui::Rect::from_points(&projected_corners);

    if !rect.intersects(grid_bounds) {
        return;
    }

    // Relleno del piso
    painter.add(Shape::convex_polygon(
        projected_corners,
        Color32::from_rgba_premultiplied(35, 35, 40, 180),
        Stroke::new(0.5, Color32::from_rgb(50, 50, 55)),
    ));

    // Líneas del grid
    let grid_color = Color32::from_rgb(55, 55, 60);
    let mut y = -s;
    while y <= s {
        let p1 = project_orbital(Point3D::new(-s, y, 0.0), cam, center, scale);
        let p2 = project_orbital(Point3D::new(s, y, 0.0), cam, center, scale);
        painter.line_segment([p1, p2], Stroke::new(0.5, grid_color));
        y += GRID_STEP;
    }
    let mut x = -s;
    while x <= s {
        let p1 = project_orbital(Point3D::new(x, -s, 0.0), cam, center, scale);
        let p2 = project_orbital(Point3D::new(x, s, 0.0), cam, center, scale);
        painter.line_segment([p1, p2], Stroke::new(0.5, grid_color));
        x += GRID_STEP;
    }
}

// ---------------------------------------------------------------------------
// Ejes de coordenadas
// ---------------------------------------------------------------------------

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
        painter.text(
            tip_screen,
            egui::Align2::CENTER_CENTER,
            label,
            font_id.clone(),
            color,
        );
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
        let (pos, _) = project_orbital_depth(Point3D::origin(), &cam, center, 1.0);
        assert!((pos.x - 100.0).abs() < 1e-6);
        assert!((pos.y - 100.0).abs() < 1e-6);
    }
}
