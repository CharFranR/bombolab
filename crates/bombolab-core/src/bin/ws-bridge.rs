// ---------------------------------------------------------------------------
// ws-bridge — Puente WebSocket → Serial para el FABRI Creator.
// ---------------------------------------------------------------------------

use std::sync::Arc;

use bombolab_core::communication::ArduinoNano;
use bombolab_core::robot::fabri_creator;

use futures_util::{SinkExt, StreamExt};
use serde::{Deserialize, Serialize};
use tokio::net::TcpListener;
use tokio::sync::Mutex;

// ─── Protocolo ─────────────────────────────────────────────────────────────

#[derive(Deserialize)]
struct QMessage {
 #[serde(rename = "type")]
 msg_type: String,
 joints: Vec<f64>,
 #[serde(default)]
 gripper: u8,
}

#[derive(Serialize)]
struct OkMessage {
 #[serde(rename = "type")]
 msg_type: String,
}

// ─── Estado ────────────────────────────────────────────────────────────────

struct AppState {
 arduino: Option<ArduinoNano>,
}

// ─── Manejo de conexión ────────────────────────────────────────────────────

async fn handle_client(
 raw: tokio::net::TcpStream,
 state: Arc<Mutex<AppState>>,
) {
 let ws = match tokio_tungstenite::accept_async(raw).await {
   Ok(w) => w,
   Err(e) => {
     eprintln!("[ws-bridge] Error handshake: {e}");
     return;
   }
 };
 let (mut write, mut read) = ws.split();

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
       let servo = robot.q_to_servo(&joints);

       let mut joints_arr = [0.0_f64; 5];
       for (i, v) in servo.iter().enumerate().take(5) {
         joints_arr[i] = *v;
       }

       let cmd = match bombolab_core::communication::ServoCommand::new(joints_arr, gripper) {
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
                 serde_json::to_string(&OkMessage { msg_type: "ok".to_string() })
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
             serde_json::to_string(&OkMessage { msg_type: "ok".to_string() })
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
   tokio::spawn(async move {
     handle_client(stream, state).await;
   });
 }
}
