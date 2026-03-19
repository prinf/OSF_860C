# TSDZ8 OSF - Technical Notes from Endless Sphere Thread
## Source: https://endless-sphere.com/sphere/threads/tsdz8-osf-open-source-firmware.126904/
## Scraped: 2026-03-16 (52 pages, ~1300 posts)

---

## 1. Hardware Specifications

### MCU / Controller
- **MCU**: Infineon XMC1302 (ARM Cortex-M0, 32MHz clock) — **mstrens**, **stancecoke**
- Compared to STM32F103 (Cortex-M3, 64MHz in Lishui controllers): XMC1302 is roughly 3-4x slower — **mstrens**
- MCU has CORDIC hardware component — **mstrens**
- Flash memory: 64KB total. Program in lower part, config in upper part — **mstrens**
- 3 memory areas: (1) program itself, (2) hex from configurator, (3) display-modifiable params saved to flash — **mstrens**
- Controller has 3 phase current shunts (low-side MOSFETs) — **stancecoke**, **mstrens**
- Some components rated for 60V max — **mstrens** (re: 72V question)

### Motor
- **4 pole pairs** (8 physical poles) vs 8 pole pairs (16 physical poles) on TSDZ2 — **mstrens** (note: mstrens says "4 poles" in thread/comments but means 4 pole pairs; code has `MOTOR_POLE_PAIRS = 4`). ERPS is half of TSDZ2 at same mechanical RPM, so all ERPS thresholds are halved.
- Rated 750W nominal
- Motor announced for 4700 RPM — **mstrens**
- Internal resistance R ~ 0.1 Ohm per phase (estimated), inductance L ~ 150uH per phase (estimated) — **mstrens** (asked community to measure)
- To measure R: disconnect 3 motor wires from controller, measure between 2 wires, divide by 2 — **mstrens**
- Rotor construction similar to BBSHD — **phobos**

### Torque Sensor
- Uses 2 disks, each with a coil. Distance between disks affects readings — **mstrens**
- Driven with pulse: 2.5 usec pulse at 20V every 20 usec (TSDZ8) vs 2 usec for TSDZ2 — **mstrens**
- ADC 10-bit values:
  - No-load range: ~160-200 (varies per unit)
  - Max load: ~420-512 (varies per unit)
  - TSDZ2 default range was 150-300
  - **ebikestuff.eu**: measured 160 (no load) to 450 (max load)
  - **alioi11**: measured 197-202 (no load rotating), up to ~410 at 80kg
  - **Bubbels**: exceeded 500, reaching ~511-512 under hard pedaling
- Torque sensor responds nearly instantly on TSDZ8 (vs slower on TSDZ2) — **katana1234**
- Original Tongsheng firmware appears to apply heavy filtering (3-5 seconds to return to zero) — **katana1234**

### Current / Power Limits
- Default firmware: 23A — **prozyc**, **mstrens**
- EKD01 power display reads 25-30% higher than actual (ammeter verified) — **prozyc**
- At 48V: 54V × 23A = 1242W peak — **mstrens**
- 860C display originally limited to 22A / 1000W; mbrusa updated to allow 1500W / 24A — **ebikestuff.eu**
- BMS readings showed up to 24A — **ebikestuff.eu**
- Battery current measurement was reading 6.25% higher than actual — **HazzaHodgson** (recalibrated ADC)

---

## 2. PWM / Motor Control

### PWM Frequency
- **19 kHz** PWM / ADC sampling frequency — **mstrens**

### FOC / Lead Angle
- OSF does NOT use true FOC (Field Oriented Control). It uses sinusoidal commutation with advance angle tuning — **stancecoke**
- FOC multiplier parameter controls lead angle calculation
- FOC multiplier tested values: 14 (initial default), 22, 24, 28, 30, 40, 50
- Higher FOC multiplier (28-30) gives better efficiency and higher RPM — **ebikestuff.eu** (home trainer tests)
- FOC multiplier 14 is not optimal for efficiency — **ebikestuff.eu**
- Max FOC angle was limited to 13 degrees initially; increased to 25 in v0.1.32 — **mstrens**
- Efficiency at FOC=30: ~79% (trainer power / battery power, includes drivetrain losses) — **ebikestuff.eu**

### Lead Angle Calculation (v0.2.x / test6_with_pll)
- Automatic lead angle calculation based on:
  - Motor speed (RPM): 0 to 14 degrees, non-linear table — **mstrens**
  - Battery current: 0 to 15 degrees, linear — **mstrens**
  - Sum = "base lead angle", calculated at high frequency
  - Correction based on minimizing Id (unproductive current): -5 to +5 degrees (or -10 to +10), adapted slowly — **mstrens**
- FOC multiplier no longer used in v0.2.x — **mstrens**

### Field Weakening
- Original logic: FW only when duty cycle = 100%
- **HazzaHodgson** mod: hysteresis + decay scheme:
  - FW offset ramps up if duty >= ~92%
  - FW offset ramps down slowly if duty <= ~88%
  - Removed guard that disabled FW under throttle

### Current Measurement & Regulation
- Current fluctuates at 6x the electrical rotation frequency — **mstrens** (confirmed with oscilloscope on battery-motor shunt)
- v0.1.13: averaging current per full electrical rotation to smooth regulation — **mstrens**
- v0.2.x: measures all 3 phase currents (low-side shunts) at 19kHz, calculates Id — **mstrens**
- Id average calculated at 100Hz (over 190 samples), PID with Id setpoint = 0 — **mstrens**
- Optimizer at 5Hz: calculates average of abs(avg_Id), adjusts lead angle offset — **mstrens**
- Angles stored in q8.8 format (8-bit decimal) in latest versions — **mstrens**

### PLL for Rotor Position
- Hall sensor transitions are NOT equally spaced — **mstrens**
- ISR at each hall front used instead of timer capture (timer option had issues) — **mstrens**
- PLL (or alpha-beta filter) prepared for smoother rotor position estimation — **mstrens**
- With PLL, hall position calibration is less critical — **mstrens**

### Full FOC Feasibility
- Estimated execution time for FOC calculations: ~15 usec — **mstrens**
- Could fit in second half of PWM cycle after ADC conversions
- First half: rotor position calculation (~15 usec)
- No need for lead angle calculation with full FOC — **mstrens**
- Sensorless FOC example from Infineon uses PLL with CORDIC, precompiled lib — **mstrens**

---

## 3. Communication Protocols

### Display Protocol (UART)
- Controller sends 0x43 message to display — **katana1234**
- Controller also sends 0x46 message containing pedal torque (unknown unit) and power in watts × 20 — **katana1234** (reverse engineered from EKD01)
- 0x46 message: 15 bytes including checksum, mostly empty — **katana1234**
- OSF sends display data as pseudo-time (speed sensor interval) which VLCD5 converts to speed. VLCD5 applies filtering — **mstrens**
- **ebikestuff.eu** has a document for the TSDZ controller/display protocol (EN translated with DeepL)

### Compatible Displays
- **VLCD5**: original protocol, most tested
- **860C**: requires mbrusa firmware (v20.1C.5-1), Bafang UART version only — **Bubbels**
- **SW102**: works with 860C firmware branch — **ebikestuff.eu** confirmed working
- **EKD01**: supported in v0.1.20+, some display inaccuracies (power reads 25-30% high, voltage averaging lag) — **E-HP**
- **XH18**: tested by prozyc, works
- 860C Tongsheng version displays will NOT work (need Bafang UART version) — **Bubbels**

### Firmware Branches
- VLCD5 version (+ EKD01, XH18): github.com/mstrens/OSF
- 860C/SW102 version: github.com/mstrens/OSF_860C
- Main differences: ebike_app.c has many differences, motor.c fewer — **mstrens**
- 860C version never modifies MCU flash memory (display stores params internally) — **mstrens**
- VLCD5 version stores some params in MCU flash — **mstrens**

---

## 4. Torque Sensor Logic Variants

### Spider Logic (mstrens original)
- More intuitive feel per **prozyc** (preferred it over Katana)
- v0.1.25 "spider" considered best by some testers — **prozyc**

### Katana Logic (katana1234)
- Added exponential ("expo") curve to torque input
- Allows different sensitivity per assist level (like Bosch modes)
- Negative expo = more assistance at low input force
- Positive expo = less assistance at low input force
- Max assistance unchanged regardless of expo — **mstrens**

### Torque Filtering
- OSF default: minimal filtering: `ui16_adc_pedal_torque_delta = (delta + delta_temp) >> 1` — **mstrens**
- 19kHz sampling rate, very slight smoothing
- **katana1234** tested 1s averaging — helped slightly but still jerky
- Jerking primarily caused by torque variation within one pedal rotation, especially at low cadence

---

## 5. Error Codes (OSF)

| Code | Meaning |
|------|---------|
| E01 | Overvoltage |
| E02 | Torque sensor error (values out of range) |
| E03 | Cadence sensor error |
| E04 | Motor blocked |
| E05 | Throttle error |
| E06 | Overtemperature |
| E07 | Battery overcurrent |
| E08 | Speed sensor error |
| E09 | EEPROM write error / Motor check (shared) |

- **mstrens**: Error codes differ from VLCD5 manual meanings
- XH18 display maps differently (E06 blinking = E01, E03 blinking = E05, etc.)
- v0.2.x added per-phase overcurrent checks (3 shunt currents) generating E07 — **mstrens**
- Overcurrent delay parameter: value × 25ms tolerance window; 0 = disabled — **mstrens**

---

## 6. Configuration Key Parameters

### Torque Sensor Setup
- **Torque ADC offset**: max ADC value during full pedal rotation with no weight + margin (~10)
- **Torque ADC max**: max ADC value at max expected pedal force
- **Torque ADC step**: calculated as `weight_kg × 167 / (ADC_at_weight - offset) × 160 / (ADC_max - offset)` — **mstrens**
- Torque sensor error triggered if ADC > 500 (increased to 650 in v0.1.20) — **mstrens**

### Motor Tuning
- **Acceleration/Deceleration**: default 35/35; reducing jerk: acc 45-55, dec 70-90 — **prozyc**, **ebikestuff.eu**
- **FOC multiplier**: 23-30 recommended range (v0.1.x)
- **Field weakening**: enable for higher top speed
- **Start boost**: available, can be too aggressive — **prozyc**

### Flashing Procedure
- J-Link required for controller
- **Critical**: use "manual programming" for 2nd file to avoid erasing 1st file — **mstrens**
- Full chip erase before fresh install helps resolve jerking issues — **prozyc**, **Bubbels**
- J-Link connection: 4 wires (VRef, SWDIO, SWCLK, GND) or 3 wires (no VRef) with battery power — **prozyc**
- **Warning**: uc_probe_monitoring with mismatched .elf/.hex can destroy controller — **mstrens**
- Do NOT flash controller with battery power on via J-Link power — **gibbeer**

### 860C Display Flashing
- Requires USB-to-UART + buck converter (30V+ needed to power display) — **HazzaHodgson**
- J-Link V9 UART mode possible but unreliable — **HazzaHodgson**
- Pin connections: RX-TX, TX-RX, GND-GND, 5V-5V (through buck converter from 30V+)

---

## 7. Known Issues & Fixes

### Controller Destruction
- 3 controllers destroyed early in development — **mstrens**, **ebikestuff.eu**, **dameri**
- mstrens: only torque sensor driver circuit damaged, not power stage
- Oscilloscope showed: 5V signal at few kHz instead of expected 20V 2.5usec pulse — **mstrens**
- uc_probe_monitoring with wrong .elf identified as likely cause for mstrens — **mstrens**

### Jerking / Fluctuating Assistance
- Primary cause: torque sensor variation within pedal rotation + fast response — **mstrens**, **katana1234**
- Fixes that help:
  - Increase deceleration to 70-90 — **prozyc**, **ebikestuff.eu**
  - Set correct torque ADC range (offset and max) — **ebikestuff.eu**
  - Full chip erase before reflashing — **prozyc**, **Bubbels**
  - Wait 10 seconds after power-on for TQ sensor calibration — **prozyc**
  - Use expo on torque input — **katana1234**
  - v0.1.13+ current averaging per electrical rotation — **mstrens**

### Settings Corruption After Power Cycle
- Some users reported settings corruption after battery disconnect — **prozyc**, **dameri**
- 860C version should not have this issue (doesn't write to MCU flash) — **mstrens**
- Fix: reflash config hex or reset 860C to factory defaults and reconfigure

---

## 8. Key Contributors

| Username | Role / Expertise |
|----------|-----------------|
| **mstrens** | Lead developer, OSF TSDZ8 port author, firmware architecture |
| **mbrusa** (emmebrusa) | Original OSF TSDZ2 developer, Java configurator, 860C display firmware |
| **ebikestuff.eu** | Tester, efficiency measurements, home trainer testing, protocol docs |
| **stancecoke** | EBiCS firmware author, FOC expert, motor control advisor |
| **katana1234** | Developer, expo torque logic, custom display project (touch AMOLED), reverse engineered 0x46 UART message |
| **HazzaHodgson** | Developer, FOC angle tweaks, field weakening mods, duty ramp logic, ADC recalibration, custom ESP32-S3 display project |
| **prozyc** | Extensive tester (all versions), settings optimization, flashing guidance |
| **dameri** | Long-term tester (1000+ km), early bug reporter |
| **phobos** | Motor control knowledge, FOC parameter advice |
| **alioi11** | Torque sensor curve measurements/calibration data |
| **Bubbels** | Tester, confirmed chip erase fix for jerking |

---

## 9. Software Architecture Notes

- OSF TSDZ8 is a port of OSF TSDZ2 (mbrusa version) — **mstrens**
- Key source files: `motor.c` (motor control), `ebike_app.c` (assist logic, display protocol)
- Java Configurator generates config .hex file separately from firmware .hex
- Two firmware variants maintained: VLCD5 (original protocol) and 860C (mbrusa protocol)
- PWM cycle: first half = rotor position calc, second half = ADC + FOC calc
- Assist modes: Cadence, Torque, Power, Hybrid (max of power and torque)
- Walk assist and throttle require explicit enable in config
- Current firmware (test6_with_pll branch): automatic lead angle, PLL rotor position, per-phase overcurrent protection

---

## 10. Efficiency Comparison

- OSF v0.1.31 with FOC=30: ~79% efficiency (home trainer) — **ebikestuff.eu**
- Original Tongsheng firmware: noticeably more powerful, higher efficiency — **ebikestuff.eu**
- Gap believed to be primarily in lead angle optimization — **mstrens**
- v0.2.x (test6_with_pll) aims to close this gap with automatic lead angle — **mstrens**
