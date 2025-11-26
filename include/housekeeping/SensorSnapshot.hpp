#pragma once

#include <atomic>
#include <cstdint>

struct SensorSnapshot {
  float accX = -100.0;
  float accY = -100.0;
  float accZ = -100.0;
  float gyrX = -100.0;
  float gyrY = -100.0;
  float gyrZ = -100.0;
  float imuTemp = -100.0;
  float magX = -100.0;
  float magY = -100.0;
  float magZ = -100.0;
  float magTemp = -100.0;
  float temp = -100.0;
  float press = -100.0;
  float hum = -100.0;
  float voltage3v3 = -100.0;
  float current3v3 = -100.0;
  float voltage5 = -100.0;
  float current5 = -100.0;
  float voltage12 = -100.0;
  float current12 = -100.0;
  float tempC1 = -100.0;
  float thystC1 = -100.0;
  float tosC1 = -100.0;
  float tempC2 = -100.0;
  float thystC2 = -100.0;
  float tosC2 = -100.0;
  float thermocouple_c = -100.0;
  float internal_c = -100.0;

  std::atomic<uint64_t> generation{0};
};

inline SensorSnapshot gSnapshot;
