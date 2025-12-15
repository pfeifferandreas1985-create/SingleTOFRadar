# NERO SENSE - System B (Radar Unit)

**Version:** 6.1 (Final Fusion)
**Role:** High-Performance Lidar Radar Unit
**Aesthetic:** Dark Tron / Petrol Sonar

## Overview
The **NERO SENSE** is a sophisticated radar unit powered by an ESP8266, featuring a VL53L1X Time-of-Flight sensor and a round GC9A01 display. It provides real-time distance visualization with a "Dark Tron" aesthetic, multi-mode operation, and a dynamic status indication ring.

## Hardware Configuration

| Component | Model | Pin | GPIO | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **MCU** | Wemos D1 Mini | - | - | ESP8266 Core |
| **Display** | GC9A01 (240x240) | **D5** (SCLK), **D7** (MOSI), **D8** (CS), **D3** (DC), **D6** (RST) | 14, 13, 15, 0, 12 | Hardware SPI |
| **Sensor** | VL53L1X (TOF) | **D2** (SDA), **D1** (SCL) | 4, 5 | I2C Bus |
| **Servo** | SG90 / MG90S | **D4** | 2 | PWM Output |
| **Input** | TTP223 Touch | **D0** | 16 | Active High Input |

> **⚠️ CRITICAL WIRING NOTE:** 
> The Touch Sensor must be connected to **D0** and the Display Reset to **D6**. 
> *Reason:* D6 is the SPI MISO pin and cannot be used as a standard input while the display is active. D4 is used for the Servo to ensure safe boot states.

## Features

### 1. Visual Aesthetics ("Dark Tron")
- **Deep Black Background** (`0x0000`) for high contrast.
- **Bright Red Scanner** (`0xF800`) with a fading trail effect.
- **Status Ring:** A 5-pixel thick outer ring (Radius 115-119) indicating system status.
- **Dynamic Grid:** Light Grey (`0x2945`) static grid for reference.

### 2. Multi-Mode Logic
Cycle through modes by tapping the Touch Sensor (D0):

| Mode | Color | Range | Speed | Description |
| :--- | :--- | :--- | :--- | :--- |
| **PRECISION** | Orange | 300 mm | 20 ms | High-speed, close-range scanning. |
| **STANDARD** | Green | 1000 mm | 33 ms | Balanced for everyday obstacle detection. |
| **LONG RANGE** | Cyan | 4000 mm | 100 ms | Maximum sensitivity for distant objects. |

### 3. Robustness & Safety
- **Watchdog:** Automatically resets the I2C bus and sensor if data freezes for >2 seconds.
- **Wiper Logic:** Clears the radar sector ahead of the scanner to prevent visual artifacts ("ghost lines").
- **I2C Recovery:** Implements a "Nuclear" bus clear sequence on boot to unstick hung sensors.

### 4. Connectivity
- **WiFi:** Connects to Static IP `192.168.178.71`.
- **WebSocket:** Broadcasts JSON data `{"angle": 90, "distance": 450, "mode": 1}` on port 81.

## Installation

1.  **Library Dependencies:**
    -   `TFT_eSPI` (Bodmer) - *Requires User_Setup.h configuration (see below)*
    -   `VL53L1X` (Pololu)
    -   `Servo` (Built-in)
    -   `WebSockets` (Markus Sattler)

2.  **TFT_eSPI Setup:**
    Ensure your `User_Setup.h` in the library folder matches this pinout:
    ```c
    #define GC9A01_DRIVER
    #define TFT_WIDTH  240
    #define TFT_HEIGHT 240
    #define TFT_MOSI 13 // D7
    #define TFT_SCLK 14 // D5
    #define TFT_CS   15 // D8
    #define TFT_DC    0 // D3
    #define TFT_RST  12 // D6 (IMPORTANT!)
    #define SPI_FREQUENCY  27000000
    ```

3.  **Flash:**
    -   Open `nero_sense_firmware.ino` in Arduino IDE.
    -   Select Board: "LOLIN(WEMOS) D1 R2 & mini".
    -   Upload.

## Status Codes (Ring Color)
- **Blue (Blinking):** Connecting to WiFi.
- **Blue (Solid):** Online & Connected.
- **Green:** Offline / Standalone Mode.
- **Red:** Hardware Error / Sensor Freeze.
- **White (Flash):** Mode Change Confirmation.

## Contact & Copyright

**Andreas Pfeiffer**  
Seestraße 20  
78234 Engen  
Deutschland  

📧 pfeiffer.andreas1985@gmail.com

**© 2025 Andreas Pfeiffer. All Rights Reserved.**
