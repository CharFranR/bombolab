// ---------------------------------------------------------------------------
// ws-bridge — Puente WebSocket → Serial para el FABRI Creator.
// ---------------------------------------------------------------------------

use std::sync::Arc;
use std::time::Duration;

use bombolab_core::communication::ArduinoNano;
use bombolab_core::robot::fabri_creator;

use futures_util::stream::{SplitSink, SplitStream};
use futures_util::{SinkExt, StreamExt};
use serde::{Deserialize, Serialize};
use tokio::net::TcpListener;
use tokio::sync::Mutex;
use tokio_tungstenite::tungstenite::handshake::server::{Request, Response};
use tokio_tungstenite::tungstenite::http::StatusCode;
use tokio_tungstenite::tungstenite::Message;
use tokio_tungstenite::WebSocketStream;

// ─── Protocolo ─────────────────────────────────────────────────────────────

#[derive(Deserialize)]
struct QMessage {
 #[serde(rename = "type")]
 msg_type: String,
 joints: Vec<f64>,
 #[serde(default)]
 gripper: u8,
}

#[derive(Deserialize)]
struct AuthMessage {
 #[serde(rename = "type")]
 msg_type: String,
 token: String,
}

#[derive(Serialize)]
struct OkMessage {
 #[serde(rename = "type")]
 msg_type: String,
 /// "serial" cuando el comando se envió al hardware, "simulation" en caso contrario.
 mode: String,
}

// ─── Estado ────────────────────────────────────────────────────────────────

struct AppState {
 arduino: Option<ArduinoNano>,
}

// ─── Conversión q → comando ────────────────────────────────────────────────

/// Construye un `ServoCommand` desde ángulos cinemáticos q (radianes).
///
/// `Robot::q_to_servo` devuelve ángulos de servo en radianes, pero
/// `ServoCommand::new` valida grados — se convierte antes de validar.
/// Rechaza q con longitud ≠ 5 o con valores no finitos (NaN/±Inf).
fn build_servo_command(
 robot: &bombolab_core::robot::Robot,
 joints: &[f64],
 gripper: u8,
) -> Result<bombolab_core::communication::ServoCommand, String> {
 if joints.len() != 5 {
   return Err("joints must contain exactly 5 values".to_string());
 }
 for &j in joints {
   if !j.is_finite() {
     return Err("joint values must be finite".to_string());
   }
 }

 let servo = robot.q_to_servo(joints);

 let mut joints_arr = [0.0_f64; 5];
 for (i, v) in servo.iter().enumerate().take(5) {
   joints_arr[i] = v.to_degrees();
 }

 bombolab_core::communication::ServoCommand::new(joints_arr, gripper)
   .map_err(|e| e.to_string())
}

// ─── Seguridad ─────────────────────────────────────────────────────────────

/// Permite conexiones sin header `Origin` (clientes no-navegador) y, si la
/// allowlist no está vacía, solo orígenes listados en `WS_BRIDGE_ALLOW_ORIGIN`.
fn origin_allowed(origin: Option<&str>, allowlist: &[String]) -> bool {
 match origin {
   None => true,
   Some(o) => allowlist.is_empty() || allowlist.iter().any(|a| a == o),
 }
}

/// Autentica al cliente con el primer mensaje `{"type":"auth","token":"…"}`.
///
/// Si no llega ningún mensaje válido dentro de 5 segundos, desconecta.
async fn authenticate(
 write: &mut SplitSink<WebSocketStream<tokio::net::TcpStream>, Message>,
 read: &mut SplitStream<WebSocketStream<tokio::net::TcpStream>>,
 expected: &str,
) -> Result<(), ()> {
 let first = match tokio::time::timeout(Duration::from_secs(5), read.next()).await {
   Err(_) => {
     eprintln!("[ws-bridge] Cliente no autenticado en 5s — desconectado");
     return Err(());
   }
   Ok(None) | Ok(Some(Err(_))) => return Err(()),
   Ok(Some(Ok(m))) => m,
 };

 if !first.is_text() {
   return Err(());
 }

 let text = first.to_text().unwrap_or("");
 let parsed: Result<AuthMessage, _> = serde_json::from_str(text);
 let ok = matches!(
   parsed,
   Ok(AuthMessage { msg_type, token }) if msg_type == "auth" && token == expected
 );

 if !ok {
   let _ = write
     .send(Message::Text(r#"{"type":"error","msg":"autenticación requerida"}"#.into()))
     .await;
 }
 if ok { Ok(()) } else { Err(()) }
}

// ─── Manejo de conexión ────────────────────────────────────────────────────

async fn handle_client(
 raw: tokio::net::TcpStream,
 state: Arc<Mutex<AppState>>,
 auth_token: Option<String>,
 allow_origins: Vec<String>,
) {
 let ws = match tokio_tungstenite::accept_hdr_async(raw, |req: &Request, mut response: Response| {
   let origin = req.headers().get("origin").and_then(|v| v.to_str().ok());
   if !origin_allowed(origin, &allow_origins) {
     *response.status_mut() = StatusCode::FORBIDDEN;
     // El error de handshake es un `ErrorResponse` (body Option<String>).
     return Err(response.map(|_| None));
   }
   Ok(response)
 })
 .await
 {
   Ok(w) => w,
   Err(e) => {
     eprintln!("[ws-bridge] Error handshake: {e}");
     return;
   }
 };
 let (mut write, mut read) = ws.split();

 if let Some(expected) = auth_token {
   if authenticate(&mut write, &mut read, &expected).await.is_err() {
     return;
   }
 }

 println!("[ws-bridge] Cliente conectado");
 let robot = fabri_creator();

 while let Some(msg) = read.next().await {
   let msg = match msg {
     Ok(m) => m,
     Err(_) => break,
   };

   if !msg.is_text() {
     continue;
   }

   let text = msg.to_text().unwrap_or("");
   let parsed: Result<QMessage, _> = serde_json::from_str(text);

   match parsed {
     Ok(QMessage { msg_type, joints, gripper }) if msg_type == "q" => {
       let cmd = match build_servo_command(&robot, &joints, gripper) {
         Ok(c) => c,
         Err(e) => {
           let err = format!(r#"{{"type":"error","msg":"{e}"}}"#);
           let _ = write.send(tokio_tungstenite::tungstenite::Message::Text(err.into())).await;
           continue;
         }
       };

       let mut st = state.lock().await;
       if let Some(ref mut arduino) = st.arduino {
         match arduino.send_and_verify(&cmd) {
           Ok(_) => {
             let _ = write
               .send(tokio_tungstenite::tungstenite::Message::Text(
                 serde_json::to_string(&OkMessage { msg_type: "ok".to_string(), mode: "serial".to_string() })
                   .unwrap()
                   .into(),
               ))
               .await;
           }
           Err(e) => {
             let err = format!(r#"{{"type":"error","msg":"{e}"}}"#);
             let _ = write.send(tokio_tungstenite::tungstenite::Message::Text(err.into())).await;
           }
         }
       } else {
         println!("[ws-bridge] Simulación — q={joints:?}, gripper={gripper}");
         let _ = write
           .send(tokio_tungstenite::tungstenite::Message::Text(
             serde_json::to_string(&OkMessage { msg_type: "ok".to_string(), mode: "simulation".to_string() })
               .unwrap()
               .into(),
           ))
           .await;
       }
     }
     _ => {
       let _ = write
         .send(tokio_tungstenite::tungstenite::Message::Text(
           r#"{"type":"error","msg":"formato inválido"}"#.into(),
         ))
         .await;
     }
   }
 }
 println!("[ws-bridge] Cliente desconectado");
}

// ─── Main ──────────────────────────────────────────────────────────────────

#[tokio::main]
async fn main() {
 let port = std::env::var("PORT").unwrap_or_else(|_| "8080".to_string());
 let serial_port = std::env::var("SERIAL_PORT").ok();
 let auth_token = std::env::var("WS_BRIDGE_TOKEN")
   .ok()
   .filter(|t| !t.is_empty());
 let allow_origins: Vec<String> = std::env::var("WS_BRIDGE_ALLOW_ORIGIN")
   .ok()
   .map(|s| {
     s.split(',')
       .map(|o| o.trim().to_string())
       .filter(|o| !o.is_empty())
       .collect()
   })
   .unwrap_or_default();

 if auth_token.is_some() {
   println!("[ws-bridge] Autenticación por token habilitada (WS_BRIDGE_TOKEN)");
 }
 if !allow_origins.is_empty() {
   println!("[ws-bridge] Orígenes permitidos: {allow_origins:?}");
 }

 let arduino = if let Some(ref port_name) = serial_port {
   match ArduinoNano::connect(port_name) {
     Ok(a) => {
       println!("[ws-bridge] Conectado a {port_name}");
       Some(a)
     }
     Err(e) => {
       eprintln!("[ws-bridge] Error conectando a {port_name}: {e}");
       eprintln!("[ws-bridge] Modo simulación");
       None
     }
   }
 } else {
   println!("[ws-bridge] SERIAL_PORT no configurado — modo simulación");
   None
 };

 let state = Arc::new(Mutex::new(AppState { arduino }));
 let addr = format!("127.0.0.1:{port}");

 let listener = match TcpListener::bind(&addr).await {
   Ok(l) => l,
   Err(e) => {
     eprintln!("[ws-bridge] Error creando servidor en {addr}: {e}");
     return;
   }
 };

 println!("[ws-bridge] Servidor WebSocket en ws://{addr}");

 while let Ok((stream, peer)) = listener.accept().await {
   println!("[ws-bridge] Conexión desde {peer}");
   let state = state.clone();
   let auth_token = auth_token.clone();
   let allow_origins = allow_origins.clone();
   tokio::spawn(async move {
     handle_client(stream, state, auth_token, allow_origins).await;
   });
 }
}

#[cfg(test)]
mod tests {
 use super::*;

 fn assert_deg(actual: f64, expected: f64) {
   assert!(
     (actual - expected).abs() < 1e-9,
     "esperado {expected}°, obtenido {actual}°"
   );
 }

 #[test]
 fn test_build_servo_command_home_pose_in_degrees() {
   // En home (q = [0; 5]) los offsets son [90, 90, 81, 95, 60] grados,
   // todos dentro de [10, 170] → el comando debe ser válido.
   let robot = fabri_creator();
   let cmd = build_servo_command(&robot, &[0.0; 5], 90).unwrap();
   assert_deg(cmd.joints[0], 90.0);
   assert_deg(cmd.joints[1], 90.0);
   assert_deg(cmd.joints[2], 81.0);
   assert_deg(cmd.joints[3], 95.0);
   assert_deg(cmd.joints[4], 60.0);
   assert_eq!(cmd.gripper, 90);
 }

 #[test]
 fn test_build_servo_command_rejects_out_of_range_degrees() {
   // q que mapea a un servo fuera de [10, 170] grados → error.
   let robot = fabri_creator();
   let result = build_servo_command(&robot, &[2.5; 5], 90);
   assert!(result.is_err());
 }

 #[test]
 fn test_build_servo_command_rejects_wrong_length() {
   let robot = fabri_creator();
   assert!(build_servo_command(&robot, &[], 90).is_err());
   assert!(build_servo_command(&robot, &[0.0; 4], 90).is_err());
   assert!(build_servo_command(&robot, &[0.0; 6], 90).is_err());
 }

 #[test]
 fn test_build_servo_command_rejects_non_finite() {
   let robot = fabri_creator();
   assert!(build_servo_command(&robot, &[f64::NAN, 0.0, 0.0, 0.0, 0.0], 90).is_err());
   assert!(build_servo_command(&robot, &[f64::INFINITY, 0.0, 0.0, 0.0, 0.0], 90).is_err());
   assert!(build_servo_command(&robot, &[f64::NEG_INFINITY, 0.0, 0.0, 0.0, 0.0], 90).is_err());
 }

 #[test]
 fn test_origin_allowed_empty_list_accepts_all() {
   let allowlist: Vec<String> = vec![];
   assert!(origin_allowed(Some("http://evil.example"), &allowlist));
   assert!(origin_allowed(None, &allowlist));
 }

 #[test]
 fn test_origin_allowed_allowlist() {
   let allowlist = vec!["http://localhost:5173".to_string()];
   assert!(origin_allowed(None, &allowlist));
   assert!(origin_allowed(Some("http://localhost:5173"), &allowlist));
   assert!(!origin_allowed(Some("http://evil.example"), &allowlist));
   assert!(!origin_allowed(Some("http://localhost:5174"), &allowlist));
 }
}
