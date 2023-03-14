#pragma once
#ifdef MP_USE_DEBUG
#ifdef ARDUINO
#define MP_DEBUGF(msg, ...) Serial.printf(msg, ## __VA_ARGS__);
#define MP_DEBUGLN(msg) Serial.println(msg);
#endif
#else
#define MP_DEBUGF(msg, ...) do {(void)0;} while (0)
#define MP_DEBUGLN(msg) do {(void)0;} while (0)
#endif
