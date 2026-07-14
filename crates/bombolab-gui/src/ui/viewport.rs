// ---------------------------------------------------------------------------
// Módulo viewport — Renderizado 3D estilo "esqueleto" (wireframe).
//
// Proporciona una función de dibujo que toma una lista de puntos 3D (las
// posiciones de las articulaciones calculadas por cinemática directa) y las
// proyecta a 2D usando una proyección isométrica simple.
//
// El resultado visual son líneas gruesas (eslabones) conectando círculos
// (articulaciones) sobre un fondo oscuro, similar a un wireframe.
// ---------------------------------------------------------------------------

use egui::{Color32, Painter, Pos2, Rect, Stroke};

// ---------------------------------------------------------------------------
// Punto 3D
// ---------------------------------------------------------------------------

/// Representa un punto en el espacio 3D del robot.
///
///
/// Se usa `f32` para mantener compatibilidad con las funciones de dibujo de
/// egui sin conversiones repetidas. La conversión desde `nalgebra::Vector3<f64>`
/// se hace en el punto de llamada.
#[derive(Clone, Copy, Debug, Default)]
pub struct Point3D {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Point3D {
    /// Crea un nuevo punto 3D.
    pub const fn new(x: f32, y: f32, z: f32) -> Self {
        Self { x, y, z }
    }

    /// Crea un punto en el origen (0, 0, 0).
    pub const fn origin() -> Self {
        Self {
            x: 0.0,
            y: 0.0,
            z: 0.0,
        }
    }

    /// Distancia euclidiana desde el origen.
    pub fn magnitude(&self) -> f32 {
        (self.x * self.x + self.y * self.y + self.z * self.z).sqrt()
    }
}

// ---------------------------------------------------------------------------
// Proyección isométrica
// ---------------------------------------------------------------------------

/// Relación de aspecto para la proyección isométrica.
///
/// Un valor de 0.5 significa que el eje Y de la pantalla se comprime a la
/// mitad, dando la sensación de profundidad.
const ISO_Y_FACTOR: f32 = 0.5;

/// Proyecta un punto 3D a coordenadas 2D de pantalla usando proyección
/// isométrica simple.
///
/// La fórmula clásica es:
///   screen_x = center.x + (point.x - point.y) * scale
///   screen_y = center.y + (point.x + point.y) * ISO_Y_FACTOR * scale - point.z * scale
///
/// Esto da una vista en la que los ejes X e Y se abren en abanico y el eje Z
/// apunta hacia arriba, ideal para visualizar brazos robóticos sobre una mesa.
fn project_isometric(point: Point3D, center: Pos2, scale: f32) -> Pos2 {
    let screen_x = center.x + (point.x - point.y) * scale;
    let screen_y = center.y + (point.x + point.y) * ISO_Y_FACTOR * scale - point.z * scale;
    Pos2::new(screen_x, screen_y)
}

// ---------------------------------------------------------------------------
// Cálculo de escala automática
// ---------------------------------------------------------------------------

/// Calcula un factor de escala para que el robot quepa dentro del viewport.
///
/// Toma el punto más lejano del origen y ajusta la escala para que ocupe
/// aproximadamente el 40 % del tamaño del viewport. Si todos los puntos
/// están en el origen, retorna una escala por defecto.
fn compute_scale(points: &[Point3D], viewport_size: f32) -> f32 {
    if viewport_size <= 0.0 {
        return 1.0;
    }

    // Encontrar la distancia máxima desde el origen
    let max_dist = points
        .iter()
        .map(|p| p.magnitude())
        .fold(0.0f32, f32::max);

    if max_dist < 1e-6 {
        // No hay puntos significativos, usar escala por defecto
        return viewport_size * 0.015;
    }

    // Escala para que el robot ocupe ~40 % del viewport
    let target_size = viewport_size * 0.40;
    target_size / max_dist
}

// ---------------------------------------------------------------------------
// Renderizado principal
// ---------------------------------------------------------------------------

/// Renderiza el robot en estilo esqueleto (wireframe) sobre el `Painter` de egui.
///
/// # Argumentos
///
/// * `painter`    — El painter de egui sobre el que dibujar.
/// * `rect`       — El área disponible para el viewport.
/// * `points`     — Lista de puntos 3D del robot. El orden debe ser cadena
///                  cinemática: base → joint1 → joint2 → ... → end-effector.
/// * `joint_color`- Color de relleno de las articulaciones.
/// * `link_color` — Color de los eslabones (líneas entre articulaciones).
/// * `link_width` — Grosor de las líneas en píxeles.
/// * `joint_radius` — Radio de los círculos de articulación en píxeles.
///
/// # Comportamiento
///
/// 1. Dibuja un plano de suelo difuso como referencia visual.
/// 2. Proyecta todos los puntos 3D a 2D usando proyección isométrica.
/// 3. Traza líneas gruesas entre puntos consecutivos (eslabones).
/// 4. Dibuja círculos rellenos en cada punto (articulaciones).
/// 5. La base se pinta en gris, el end-effector en verde, y las articulaciones
///    intermedias en `joint_color`.
///
/// Si `points` está vacío o solo contiene el origen, la función no dibuja nada
/// (quien llama debe mostrar un placeholder).
pub fn draw_robot_skeleton(
    painter: &Painter,
    rect: Rect,
    points: &[Point3D],
    joint_color: Color32,
    link_color: Color32,
    link_width: f32,
    joint_radius: f32,
) {
    if points.len() < 2 {
        return; // No hay suficiente geometría para dibujar
    }

    let viewport_size = rect.width().min(rect.height());
    let scale = compute_scale(points, viewport_size);

    let center = rect.center();

    // 1. Dibujar plano de suelo (grid de referencia)
    draw_ground_grid(painter, center, scale, rect);

    // 2. Proyectar todos los puntos a 2D
    let projected: Vec<Pos2> = points
        .iter()
        .map(|p| project_isometric(*p, center, scale))
        .collect();

    // 3. Dibujar eslabones (líneas entre articulaciones consecutivas)
    for window in projected.windows(2) {
        let stroke = Stroke::new(link_width, link_color);
        painter.line_segment([window[0], window[1]], stroke);
    }

    // 4. Dibujar articulaciones (círculos)
    let num_points = projected.len();
    for (i, pos) in projected.iter().enumerate() {
        // Determinar color según la posición en la cadena
        let color = if i == 0 {
            // Base del robot — gris
            Color32::from_rgb(160, 160, 160)
        } else if i == num_points - 1 {
            // End-effector — verde
            Color32::from_rgb(80, 220, 80)
        } else {
            // Articulación intermedia
            joint_color
        };

        // Relleno
        painter.circle_filled(*pos, joint_radius, color);
        // Borde sutil para mejorar legibilidad
        painter.circle_stroke(*pos, joint_radius, Stroke::new(1.0, Color32::WHITE.gamma_multiply(0.3)));
    }
}

// ---------------------------------------------------------------------------
// Plano de suelo (grid)
// ---------------------------------------------------------------------------

/// Dibuja un pequeño grid en el origen para dar sensación de profundidad.
fn draw_ground_grid(painter: &Painter, center: Pos2, scale: f32, rect: Rect) {
    // El grid se dibuja en el plano z=0, formando un cuadrado alrededor del origen
    let grid_size = 3.0; // medio ancho del grid en unidades del mundo
    let grid_color = Color32::from_rgb(60, 60, 60);
    let step = 1.0; // separación entre líneas del grid

    // Proyectar las 4 esquinas del grid para determinar el área en pantalla
    let corners = [
        Point3D::new(-grid_size, -grid_size, 0.0),
        Point3D::new(grid_size, -grid_size, 0.0),
        Point3D::new(grid_size, grid_size, 0.0),
        Point3D::new(-grid_size, grid_size, 0.0),
    ];
    let projected_corners: Vec<Pos2> = corners
        .iter()
        .map(|p| project_isometric(*p, center, scale))
        .collect();

    // Solo dibujar si el grid está dentro del viewport (culling básico)
    let grid_bounds = egui::Rect::from_points(&projected_corners);
    if !rect.intersects(grid_bounds) {
        return;
    }

    // Líneas paralelas al eje X (en el plano del suelo)
    let mut y = -grid_size;
    while y <= grid_size {
        let p1 = project_isometric(Point3D::new(-grid_size, y, 0.0), center, scale);
        let p2 = project_isometric(Point3D::new(grid_size, y, 0.0), center, scale);
        painter.line_segment([p1, p2], Stroke::new(0.5, grid_color));
        y += step;
    }

    // Líneas paralelas al eje Y (en el plano del suelo)
    let mut x = -grid_size;
    while x <= grid_size {
        let p1 = project_isometric(Point3D::new(x, -grid_size, 0.0), center, scale);
        let p2 = project_isometric(Point3D::new(x, grid_size, 0.0), center, scale);
        painter.line_segment([p1, p2], Stroke::new(0.5, grid_color));
        x += step;
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
    fn test_compute_scale_default() {
        // Todos los puntos en el origen → escala por defecto
        let points = vec![Point3D::origin(), Point3D::origin()];
        let scale = compute_scale(&points, 500.0);
        assert!(scale > 0.0);
        // 500 * 0.015 = 7.5
        assert!((scale - 7.5).abs() < 1e-6);
    }

    #[test]
    fn test_compute_scale_dynamic() {
        // Punto a distancia 10 del origen → 500 * 0.4 / 10 = 20
        let points = vec![Point3D::origin(), Point3D::new(10.0, 0.0, 0.0)];
        let scale = compute_scale(&points, 500.0);
        assert!((scale - 20.0).abs() < 1e-6);
    }

    #[test]
    fn test_compute_scale_zero_viewport() {
        let points = vec![Point3D::origin(), Point3D::new(1.0, 0.0, 0.0)];
        let scale = compute_scale(&points, 0.0);
        assert!((scale - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_project_isometric_origin() {
        let center = Pos2::new(100.0, 100.0);
        let projected = project_isometric(Point3D::origin(), center, 1.0);
        assert!((projected.x - 100.0).abs() < 1e-6);
        assert!((projected.y - 100.0).abs() < 1e-6);
    }
}
