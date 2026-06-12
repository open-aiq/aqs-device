# Device Firmware — Air Quality Monitor

ESP32 firmware for an air-quality monitor that reads particulate matter, temperature,
and humidity, drives a 16×2 I2C LCD, handles four push buttons via interrupts, and
reports over WiFi.

## Hardware

- **MCU:** ESP32 (DOIT ESP32 DevKit v1)
- **PM sensor:** Plantower PMS5003 (UART, on `Serial2`)
- **Temp/Humidity:** DHT22
- **Display:** 16×2 character LCD over I2C (PCF8574 backpack, address `0x3F`)
- **Input:** 4 push buttons (Right / Left / Settings / Boot)

## Pin map

| Function            | GPIO        | Notes                          |
|---------------------|-------------|--------------------------------|
| I2C SDA / SCL       | 21 / 22     | LCD                            |
| PMS5003 RX / TX     | 16 / 17     | `Serial2`, 9600 baud           |
| DHT22 data          | 14          |                                |
| Button: Right       | 34          | input-only pin, FALLING edge   |
| Button: Left        | 35          | input-only pin, FALLING edge   |
| Button: Settings    | 36          | input-only pin, FALLING edge   |
| Button: Boot        | 39          | input-only pin, FALLING edge   |

> Note: GPIO 34/35/36/39 are input-only and have **no internal pull-ups** — make sure
> each button has an external pull-up resistor.

## Project layout

```
.
├── platformio.ini          # build config, board, library deps
├── src/main.cpp            # firmware source
├── include/
│   ├── secrets.h           # WiFi credentials (gitignored — create from template)
│   └── secrets.example.h   # template to copy
└── README.md
```

## Setup

This is a [PlatformIO](https://platformio.org/) project.

1. Copy the secrets template and fill in your WiFi details:

   ```bash
   cp include/secrets.example.h include/secrets.h
   # then edit include/secrets.h
   ```

2. Build:

   ```bash
   pio run
   ```

3. Upload to the board:

   ```bash
   pio run -t upload
   ```

4. Open the serial monitor (9600 baud):

   ```bash
   pio device monitor
   ```

## Configuration

- WiFi credentials live in `include/secrets.h` (`WIFI_SSID`, `WIFI_PASSWORD`).
- Board, monitor speed, and library dependencies are set in `platformio.ini`.
