#include "comms.h"
#include <ArduinoJson.h>
#include <ArduinoWebsockets.h>
#include "config.h"
#include "imu.h"
#include "leds.h"
#include "servos.h"

using namespace websockets;

namespace comms {
namespace {

WebsocketsServer server;
WebsocketsClient client;
bool is_connected = false;
bool mpu_stream = false;
uint32_t last_stream = 0;

// ----- Envío -----

void sendRaw(const char* data, size_t len) {
  if (!is_connected) return;
  client.send(data, len);
  leds::markActivity(leds::Activity::Sending);
}

void sendJson(const JsonDocument& doc) {
  char buf[512];
  if (measureJson(doc) >= sizeof(buf)) {
    LOGI("JSON demasiado grande, no se envía\n");
    return;
  }
  sendRaw(buf, serializeJson(doc, buf, sizeof(buf)));
}

void sendStatus(const char* status) {
  JsonDocument doc;
  doc["status"] = status;
  sendJson(doc);
}

void sendError(const char* msg) {
  LOGI("Error: %s\n", msg);
  JsonDocument doc;
  doc["status"] = "error";
  doc["error"] = msg;
  sendJson(doc);
}

void sendTelemetry() {
  if (!imu::ok()) return sendError("IMU no disponible");
  JsonDocument doc;
  imu::writeTelemetry(doc);
  sendJson(doc);
}

// ----- Ángulos -----

// Acepta [a1..a12] o [[a,b,c] x4]. Valida todo antes de mover:
// un valor no numérico en V5 se convertía en 0° y movía el servo.
bool parseAngles(JsonArrayConst arr, float (&out)[cfg::NUM_SERVOS], bool& nested) {
  nested = arr.size() == cfg::NUM_LEGS && arr[0].is<JsonArrayConst>();

  if (!nested) {
    if (arr.size() != cfg::NUM_SERVOS) return false;
    for (uint8_t i = 0; i < cfg::NUM_SERVOS; i++) {
      if (!arr[i].is<float>()) return false;
      out[i] = arr[i].as<float>();
    }
  } else {
    for (uint8_t leg = 0; leg < cfg::NUM_LEGS; leg++) {
      JsonArrayConst joints = arr[leg];
      if (joints.size() != cfg::JOINTS_PER_LEG) return false;
      for (uint8_t j = 0; j < cfg::JOINTS_PER_LEG; j++) {
        if (!joints[j].is<float>()) return false;
        out[leg * cfg::JOINTS_PER_LEG + j] = joints[j].as<float>();
      }
    }
  }
  for (float a : out) {
    if (isnan(a) || isinf(a)) return false;
  }
  return true;
}

void handleAngles(JsonArrayConst arr) {
  float angles[cfg::NUM_SERVOS];
  bool nested;
  if (!parseAngles(arr, angles, nested)) {
    return sendError("formato de angulos invalido");
  }
  servos::setAll(angles);
  if (cfg::ACK_ANGLE_PACKETS) {
    sendStatus(nested ? "angles_received_nested" : "angles_received_linear");
  }
}

// ----- Comandos -----

// "cmd:set_offsets:ax,ay,az,gx,gy,gz"
bool parseOffsets(const char* s, float (&v)[6]) {
  for (int i = 0; i < 6; i++) {
    char* end;
    v[i] = strtof(s, &end);
    if (end == s) return false;
    s = end;
    if (i < 5) {
      if (*s != ',') return false;
      s++;
    }
  }
  return true;
}

void setMpuStream(bool on, bool ack) {
  mpu_stream = on;
  LOGI("Stream MPU %s\n", on ? "activado" : "desactivado");
  if (ack) sendStatus(on ? "MPU stream started" : "MPU stream stopped");
}

void handleCommand(const char* cmd) {
  LOGI("Comando: %s\n", cmd);
  constexpr char SET_OFFSETS[] = "set_offsets:";

  if (!strcmp(cmd, "calibrate_imu")) {
    if (!imu::ok()) return sendError("IMU no disponible");
    imu::calibrate();
    sendStatus("IMU calibrated");
  } else if (!strcmp(cmd, "reset_imu")) {
    imu::resetOrientation();
    sendStatus("IMU reset");
  } else if (!strcmp(cmd, "get_offsets")) {
    JsonDocument doc;
    imu::writeOffsets(doc);
    sendJson(doc);
  } else if (!strncmp(cmd, SET_OFFSETS, sizeof(SET_OFFSETS) - 1)) {
    float v[6];
    if (!parseOffsets(cmd + sizeof(SET_OFFSETS) - 1, v)) {
      return sendError("set_offsets requiere 6 valores: ax,ay,az,gx,gy,gz");
    }
    imu::setOffsets(&v[0], &v[3]);
    sendStatus("Offsets updated");
  } else if (!strcmp(cmd, "get_mpu_data")) {
    sendTelemetry();
  } else if (!strcmp(cmd, "start_mpu_stream")) {
    setMpuStream(true, true);
  } else if (!strcmp(cmd, "stop_mpu_stream")) {
    setMpuStream(false, true);
  } else {
    sendError("comando no reconocido");
  }
}

void handleJson(const char* data, size_t len) {
  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, data, len);
  if (err) return sendError(err.c_str());

  if (doc.is<JsonArrayConst>()) {
    handleAngles(doc.as<JsonArrayConst>());
    return;
  }

  // {"command": "..."} — igual que V5: start/stop por JSON no envían ack
  const char* command = doc["command"];
  if (!command) return sendError("JSON sin 'command'");
  if (!strcmp(command, "get_mpu_data")) sendTelemetry();
  else if (!strcmp(command, "start_mpu_stream")) setMpuStream(true, false);
  else if (!strcmp(command, "stop_mpu_stream")) setMpuStream(false, false);
  else sendError("comando no reconocido");
}

void onMessage(WebsocketsMessage msg) {
  if (!msg.isText()) return;
  leds::markActivity(leds::Activity::Receiving);

  const auto& payload = msg.data();
  const char* data = payload.c_str();
  const size_t len = payload.length();
  LOGD("<- %s\n", data);

  if (len > 4 && !strncmp(data, "cmd:", 4)) handleCommand(data + 4);
  else if (len && (data[0] == '[' || data[0] == '{')) handleJson(data, len);
  else sendError("formato no reconocido");
}

void onEvent(WebsocketsEvent event, String) {
  if (event == WebsocketsEvent::ConnectionClosed) {
    LOGI("WebSocket: cliente cerró la conexión\n");
  }
}

void setConnected(bool c) {
  if (c == is_connected) return;
  is_connected = c;
  leds::setWebSocket(c);
  if (!c) mpu_stream = false;  // el siguiente cliente decide si quiere stream
  LOGI(c ? "WebSocket CONECTADO\n" : "WebSocket DESCONECTADO\n");
}

}  // namespace

void begin() {
  server.listen(cfg::WS_PORT);
  LOGI("WebSocket escuchando en puerto %u\n", cfg::WS_PORT);
}

void loop() {
  // server.poll() no bloquea; accept() sí, por eso solo se llama si hay cliente pendiente.
  // Un cliente nuevo reemplaza al anterior (p. ej. el notebook se reconectó y
  // el socket viejo quedó medio abierto).
  if (server.poll()) {
    if (is_connected) client.close();
    client = server.accept();
    mpu_stream = false;
    client.onMessage(onMessage);
    client.onEvent(onEvent);
    setConnected(client.available());
    if (is_connected) sendStatus("Conectado al ESP32");
  }

  if (!is_connected) return;
  if (!client.available()) {
    setConnected(false);
    return;
  }
  client.poll();

  if (mpu_stream && millis() - last_stream >= cfg::IMU_STREAM_MS) {
    last_stream = millis();
    sendTelemetry();
  }
}

bool connected() { return is_connected; }

}  // namespace comms
