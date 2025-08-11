// lib/mongoose/mongoose_config.h
#pragma once

// Архитектура: Arduino ESP32
#define MG_ARCH MG_ARCH_ESP32

// Файловую систему Mongoose не используем (ESP32 Arduino сам по себе)
#define MG_ENABLE_FILE 0

// Сокеты через lwIP включены (по умолчанию так и есть)
#define MG_ENABLE_SOCKET 1

// Если понадобятся встроенные статики/ресурсы (упакованная ФС), раскомментируй:
// #define MG_ENABLE_PACKED_FS 1
// #define MG_ENABLE_POSIX_FS  0