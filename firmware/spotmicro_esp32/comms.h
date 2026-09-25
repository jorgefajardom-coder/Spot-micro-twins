#pragma once
#include <Arduino.h>

// Servidor WebSocket (un cliente a la vez) + despacho del protocolo.
// Protocolo documentado en README.md, idéntico al de V5.
namespace comms {

void begin();
void loop();       // acepta clientes, hace poll y envía el stream IMU
bool connected();

}  // namespace comms
