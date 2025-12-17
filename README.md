# ESP32 Smart Fan Controller with FreeRTOS

![Badge](https://img.shields.io/badge/Platform-ESP32-blue) ![Badge](https://img.shields.io/badge/OS-FreeRTOS-green) ![Badge](https://img.shields.io/badge/Language-C%2B%2B-orange)

## 🚀 Overview
This project implements a **Real-Time Smart Fan Controller** using the ESP32 microcontroller. Unlike traditional "Super Loop" architectures, this system leverages **FreeRTOS** to handle multiple concurrent operations—sensing, processing, user input, and display—without blocking execution.

It serves as a prototype for **Industrial IoT Edge Devices**, demonstrating how to decouple sensor data acquisition from actuator control using proper Inter-Process Communication (IPC).

### 🎮 Live Simulation
You can run the full simulation including the hardware logic and OLED display in your browser via Wokwi:
**[▶️ Run Live Simulation](https://wokwi.com/projects/448972200491964417)**

---

## ⚙️ System Architecture
The system utilizes a modular task-based architecture managed by the FreeRTOS scheduler. Data integrity is maintained using **Queues** for data transfer and **Mutexes** for shared state protection.

### Task Definition & Priority Logic
| Task Name | Priority | Periodicity | Description |
| :--- | :--- | :--- | :--- |
| **TaskTempRead** | **High (2)** | 500 ms | Reads DHT22 sensor. High priority ensures fresh data for the control loop. |
| **TaskFanControl** | **High (2)** | 200 ms | Calculates PWM duty cycle. Immediate reaction to thermal changes. |
| **TaskButtonInput** | Medium (1) | 100 ms | Polls GPIOs. Responsive UI without interrupting critical control tasks. |
| **TaskDisplay** | Low (0) | 1000 ms | Updates OLED. Lowest priority to save CPU cycles for control logic. |

---

## 📐 UML Design & Logic
The system operates in two modes (**Automatic** vs **Manual**), managed by a finite state machine.

### Data Flow Pipeline
1.  **Sensing:** `TaskTempRead` pushes raw data to a thread-safe **Queue** (`xTemperatureQueue`), decoupling the slow sensor read (~2ms) from the rest of the system.
2.  **Processing:** `TaskFanControl` pulls data from the Queue.
    *   *Auto Mode:* Maps temperature (20°C - 40°C) to Fan Speed (0% - 100%) using Linear Interpolation.
    *   *Manual Mode:* Uses user-defined speed protected by `xMutex`.
3.  **Actuation:** Generates a 25 kHz PWM signal (8-bit resolution) to drive the DC Fan.

*(See `docs/` folder for full Sequence Diagrams and State Machines)*

---

## 🛠️ Hardware Implementation
**Schematic:**
![Schematic](./docs/schematic.png)

**Pin Assignment Strategy:**
*   **GPIO 25 (PWM):** Chosen because it is not a strapping pin (prevents random fan spin at boot).
*   **GPIO 26, 27, 14 (Buttons):** Configured as `INPUT_PULLUP` to utilize internal ESP32 resistors, simplifying the circuit.
*   **GPIO 21/22 (I2C):** Default hardware I2C pins for maximum OLED refresh rate.

---

## 💾 Resource Optimization & Memory Analysis
Efficiency was a key constraint for this embedded system.

*   **No Busy Waiting:** Instead of `delay()`, the system uses `vTaskDelayUntil()`. This puts tasks into a **Blocked State**, allowing the scheduler to run the Idle task or lower-priority tasks (Display) to save power.
*   **Stack Allocation:**
    *   *Heavy Tasks (Temp/Display):* Allocated **4096 Bytes** to handle floating-point math and I2C overhead.
    *   *Light Tasks (Buttons):* Allocated **2048 Bytes** as they only require boolean logic.

---

## 📄 Documentation
For a full breakdown of the sequence diagrams, state machines, and mathematical control models, please refer to the full report:
[Download Project Report (PDF)](./docs/Project_Report.pdf)
