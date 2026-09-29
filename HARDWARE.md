# Hardware Setup & Wiring Guide

Hardware interface specifications, wiring pinouts, and operation guidelines for the FOC motor control setup using **ODrive v3.6** (STM32F405RGT6).

---

## 1. System Specifications

| Parameter | Specification | Remarks |
| :--- | :--- | :--- |
| **Main Controller** | ODrive v3.6 (STM32F405RGT6) | System Core Clock: 168 MHz |
| **Gate Driver** | TI DRV8301 | Integrated on ODrive hardware |
| **Position Sensor** | AS5048A Magnetic Rotary Encoder | SPI3 interface, 14-bit resolution (16384 CPR) |
| **Target Motor** | BLDC / Gimbal Motor (e.g., GM5208-12) | Configured for Motor 1 (M1) channel |
| **DC Bus Voltage (VBUS)** | 12V – 24V DC | Bench PSU with current limiting |

---

## 2. Wiring & Pinout Mapping

### 2.1. AS5048A Magnetic Encoder (SPI3)

The AS5048A communicates with the STM32F405 via the **SPI3** bus on the APB1 domain ($PCLK1 = 42\text{ MHz}$). The recommended SPI clock speed is $\le 10\text{ MHz}$ (configured via prescaler `/8` at $\approx 5.25\text{ MHz}$).

| AS5048A Pin | ODrive Pin / MCU Net | Header Location | Description / Notes |
| :--- | :--- | :--- | :--- |
| **VDD / 5V** | **+5V** | ODrive AUX / J4 Header | **Must be powered with 5V** (on-board 3.3V LDO mode) |
| **GND** | **GND** | ODrive GND | Common system ground reference |
| **MOSI** | PB5 (`SPI3_MOSI`) | M1 / J4 SPI Header | Master Out Slave In |
| **MISO** | PB4 (`SPI3_MISO`) | M1 / J4 SPI Header | Master In Slave Out (Sensor read data) |
| **SCK** | PB3 (`SPI3_SCK`) | M1 / J4 SPI Header | SPI Serial Clock |
| **CSN** | **GPIO 4 (PC4)** | **Header Pin 4** | **Active-low Chip Select line** |

> **Wiring Note:** Ensure reliable crimping on the 5V and GND lines. A floating ground or brownout on the 5V rail will result in intermittent `AS5048A_ERR` frame responses.

---

### 2.2. Motor Phase Connections (M1 Channel)

Connect the three motor phase leads to the screw terminals labeled for **Motor 1**:

* **Phase A:** Terminal `M1_A`
* **Phase B:** Terminal `M1_B`
* **Phase C:** Terminal `M1_C`

*(If the motor rotates opposite to the encoder direction during closed-loop operation, swap any two phase wires or invert the direction sign `g_dir` in firmware).*

---

## 3. Power-Up Sequence & Bench Safety

1. **Logic & Programming Verification:**
   * Always power logic via **ST-Link (3.3V, SWDIO, SWCLK, GND)** or USB first to verify firmware flashing and SPI telemetry before engaging high power.
2. **DC Bus Current Limit:**
   * For initial calibration, rotor alignment, and tuning, configure your DC power supply current limit to **1.0A – 1.5A**.
   * Never connect high-current LiPo batteries directly during initial commissioning.
3. **Emergency Stop Condition:**
   * If encoder read errors exceed `ENC_ERR_LIMIT`, the firmware triggers `FOC_M1_EmergencyStop()`, clamping phase duties to $0$ and resetting the DRV8301 enable line.

---

## 4. Hardware Gotchas & Best Practices

- [ ] **Supply Voltage (5V):** Ensure the AS5048A breakout board is fed with **5V**, not 3.3V, as typical breakout boards incorporate an onboard linear regulator requiring $\ge 4.5\text{V}$ for stable operation.
- [ ] **Chip Select Pin (CSN):** Verify that software pin toggling points strictly to **GPIO 4 (PC4)**. Confirm that the GPIO mode is set to `GPIO_MODE_OUTPUT_PP` with high pull-up speed.
- [ ] **Magnet Alignment:** Place a diametrically magnetized disc magnet directly above the center of the AS5048A silicon die at an air gap of **0.5 mm to 1.5 mm**. Misalignment will cause magnetic strength register faults (AGC saturation).
- [ ] **Noise Mitigation on SPI Lines:**
  - Route the SPI signal harness away from the high $\frac{dv}{dt}$ switching motor phase lines (M1_A, M1_B, M1_C).
  - For harness lengths exceeding 15 cm, twist signal pairs with GND or use shielded cable.
- [ ] **DRV8301 Enable (`EN_GATE`):** Pin `PB12` must be driven `HIGH` before activating TIM8 complementary PWM channels to release the gate driver from standby mode.
- [ ] **Hardware Dead-Time:** Complementary switching on TIM8 requires dead-time insertion (`TIM_1_8_DEADTIME_CLOCKS`) to eliminate bridge shoot-through.