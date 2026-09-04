#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdint.h>

// ── Pi Pico → Node B single-byte operator commands (received on Serial2 / UART2) ──
// No framing needed — each command is exactly one byte.
#define PILINK_CMD_IRRIGATE_NOW    0xA1   // Operator pressed manual irrigate
#define PILINK_CMD_EMERGENCY_STOP  0xA2   // Operator pressed emergency stop
#define PILINK_CMD_STATUS_REQUEST  0xA3   // Pi Pico requests a fresh health packet

#endif // UART_PROTOCOL_H
