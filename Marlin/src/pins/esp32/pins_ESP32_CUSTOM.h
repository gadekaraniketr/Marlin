/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Custom ESP32 Pin Configuration for XY Motion System
 * TB6600 Stepper Drivers (No Enable Pin)
 * No Temperature Sensors, No Endstops, No Homing
 */
#pragma once

#include "env_validate.h"

#define BOARD_INFO_NAME "ESP32 Custom XY"

//
// Disable I2S - We're using direct GPIO pins
//
#undef I2S_STEPPER_STREAM

//
// No Limit Switches - Disabled in Configuration.h
//
#define X_MIN_PIN                          -1
#define Y_MIN_PIN                          -1

//
// Steppers - TB6600 Drivers
// Using common ESP32 GPIO pins
// TB6600: Only STEP and DIR pins (no ENABLE pin)
//
#define X_STEP_PIN                            27  // GPIO27
#define X_DIR_PIN                             26  // GPIO26
#define X_ENABLE_PIN                          25  // GPIO25

#define Y_STEP_PIN                            33  // GPIO33
#define Y_DIR_PIN                             32  // GPIO32
#define Y_ENABLE_PIN                          X_ENABLE_PIN  // X_ENABLE_PIN

// Z axis not used but defined to prevent errors
#define Z_STEP_PIN                            -1
#define Z_DIR_PIN                             -1
#define Z_ENABLE_PIN                          -1

// Extruder not used but defined to prevent errors
#define E0_STEP_PIN                           -1
#define E0_DIR_PIN                            -1
#define E0_ENABLE_PIN                         -1

//
// Temperature Sensors - Not Used
//
// #define TEMP_0_PIN                         -1
// #define TEMP_BED_PIN                       -1

//
// Heaters / Fans - Not Used
//
// #define HEATER_0_PIN                       -1
// #define FAN0_PIN                           -1
// #define HEATER_BED_PIN                     -1

//
// SPI - Optional for SD card
//
#define SDSS                                   5
