# HumiditySense

**HumiditySense** is an STM32-based project for reading and displaying humidity levels using a humidity sensor over I2C. 
The project uses RTOS threads for structured execution and demonstrates embedded programming concepts like peripheral initialization, I2C communication, and UART output.

---

## Features

- Reads humidity data using **I2C** communication.
- Converts raw sensor data to readable values (per datasheet formula).
- Displays the humidity data via **UART** (e.g., serial console).
- Multi-threaded implementation using **CMSIS RTOS**.
- Configured for STM32 microcontrollers.

---

## Hardware Requirements

This project requires the following components:
- **Microcontroller**: STM32 (e.g., STM32L4 family or similar).
- **Humidity Sensor**: Sensirion SHT21 (or compatible I2C-based sensor).
- **Connections**:
   - VCC → 3.3V  
   - GND → Ground  
   - SDA → I2C Data Line (e.g., PB7)  
   - SCL → I2C Clock Line (e.g., PB6)
- **UART Output**:
   - UART2 for console output (e.g., connected to a serial monitor like PuTTY).

---

## How It Works

1. **I2C Communication**:  
   - Initializes I2C to communicate with the humidity sensor.  
   - Requests humidity data and retrieves it as 2-byte raw values.  
   - Converts raw data to readable percentages using the **datasheet formula**:  
     ```
     Humidity = -6 + 125 * (RawData / 65536)
     ```

2. **Multi-threaded Operation**:  
   - **Thread 1**: Reads sensor data (`StartReadSensor`).  
   - **Thread 2**: Formats and sends data over UART (`StartCheckOutp`).  
   - **Thread 3**: Reserved for future extensions (`StartSendData`).

3. **UART Output**:  
   - Prints humidity data to a serial console at 1-second intervals:
     ```
     Luftfeuchtigkeit test: 45.67 %
     ```

---

## Software Requirements

- **STM32CubeIDE** or equivalent development environment.
- STM32 HAL libraries for GPIO, I2C, UART, and RTOS support.
- Serial monitor software for viewing UART output.

