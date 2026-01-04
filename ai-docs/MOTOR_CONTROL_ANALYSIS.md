# TSDZ8 Motor Control System Analysis

## Overview

This document provides a comprehensive analysis of the TSDZ8 motor control system, covering input parameters, output parameters, control algorithms, and their interdependencies.

## System Architecture

The motor control system operates with a **dual-interrupt architecture**:
- **IRQ0 (CCU80_0)**: ADC sampling and voltage monitoring (fast, minimal processing)  
- **IRQ1 (CCU80_1)**: Motor control calculations and PWM generation (~19kHz)
- **Main Loop**: Higher-level control logic, assist modes, safety checks (25ms cycle)

## Input Parameters

### 1. Torque Sensing
- **`ui16_adc_torque`**: Raw ADC value from torque sensor (10-bit, 0-1023)
- **`ui16_adc_torque_filtered`**: Filtered torque value for noise reduction
- **`ui16_adc_torque_delta`**: Calculated torque delta (user pedaling force)
- **Location**: P2.2 (G0 CH7) 
- **Usage**: Primary input for torque-based assist modes

### 2. Pedal Cadence (PAS)
- **`ui8_pedal_cadence_RPM`**: Pedal rotation speed in RPM
- **`ui8_pas_new_transition`**: PAS state transitions for cadence calculation
- **Source**: Hall effect sensor on pedal crank
- **Usage**: Cadence-based assist, startup detection

### 3. Speed Sensing  
- **`ui16_wheel_speed_x10`**: Wheel speed × 10 (for decimal precision)
- **`ui16_motor_speed_erps`**: Motor electrical rotations per second
- **Calculation**: Based on hall sensor transitions and timing
- **Usage**: Speed limiting, efficiency optimization

### 4. Battery Monitoring
- **`ui16_adc_voltage`**: Battery voltage ADC value (10-bit)
- **`ui16_battery_voltage_filtered_x1000`**: Filtered voltage × 1000
- **`ui8_adc_battery_current_filtered`**: Battery current consumption
- **Location**: P2.4 (G1 CH6) for voltage
- **Usage**: Power calculations, undervoltage protection

### 5. Throttle (Optional)
- **`ui16_adc_throttle`**: Throttle ADC value when enabled
- **Location**: P2.5 (G1 CH7)
- **Usage**: Direct power control in throttle mode

### 6. Motor Position (Hall Sensors)
- **`ui8_hall_sensors_state`**: Current hall sensor pattern (3-bit, 0-7)
- **`ui16_hall_counter_total`**: Time between hall transitions
- **Usage**: Motor commutation, speed calculation, position feedback

### 7. Motor Phase Current
- **`ui16_adc_motor_phase_current`**: Current motor phase current
- **`ui16_adc_motor_phase_current_max`**: Maximum allowed phase current
- **Usage**: Overcurrent protection, efficiency control

### 8. System State
- **`ui8_brake_state`**: Brake sensor input (0=released, 1=pressed)
- **`ui8_riding_mode`**: Selected assist mode (power, torque, cadence, etc.)
- **`ui8_assist_level`**: Assist level from display (0-9)

### 9. Configuration Parameters (from Display)
- **`ui8_target_battery_max_power_div25`**: Max power limit ÷ 25 (e.g., 20 = 500W)
- **`ui8_riding_mode_parameter`**: Mode-specific parameter
- **`ui8_hybrid_torque_parameter`**: Hybrid mode torque factor

## Output Parameters

### 1. PWM Motor Control
- **`ui16_a`, `ui16_b`, `ui16_c`**: Three-phase PWM values for motor phases U, V, W
- **`ui8_g_duty_cycle`**: Global duty cycle (0-255, where 255 = 100%)
- **Range**: 0-1680 (PWM_COUNTER_MAX for TSDZ8)
- **Frequency**: ~19kHz PWM switching frequency
- **Purpose**: Controls motor torque and speed via three-phase AC generation

### 2. FOC (Field Oriented Control)
- **`ui8_g_foc_angle`**: Field Oriented Control angle for optimal efficiency
- **`ui8_motor_phase_absolute_angle`**: Absolute motor phase angle
- **`ui8_fw_hall_counter_offset`**: Field weakening offset for high-speed operation
- **Purpose**: Optimizes motor efficiency and performance across speed range

### 3. Current Control
- **`ui8_controller_adc_battery_current_target`**: Target battery current
- **`ui8_adc_battery_current_max`**: Maximum allowed battery current  
- **Purpose**: Limits power consumption and protects battery/controller

### 4. Dynamic Control Parameters
- **`ui8_controller_duty_cycle_target`**: Target duty cycle (usually 255)
- **`ui8_controller_duty_cycle_ramp_up_inverse_step`**: Acceleration rate
- **`ui8_controller_duty_cycle_ramp_down_inverse_step`**: Deceleration rate
- **Purpose**: Controls how quickly motor responds to power changes

## Control Flow and Dependencies

### Main Control Loop (25ms cycle)
```
ebike_control_motor() {
    1. Read sensors (torque, cadence, speed, throttle)
    2. Apply selected assist mode algorithm
    3. Calculate target current and duty cycle
    4. Apply safety limits and constraints
    5. Set controller targets for motor ISR
}
```

### Motor ISR (19kHz)
```
CCU80_1_IRQHandler() {
    1. Read hall sensors for motor position
    2. Calculate FOC angle and motor phase
    3. Apply SVM (Space Vector Modulation)
    4. Generate three-phase PWM values
    5. Apply current limiting and safety checks
    6. Update PWM registers (ui16_a, ui16_b, ui16_c)
}
```

## Assist Mode Algorithms

### 1. Power Assist Mode
**Input Dependencies**: 
- Battery voltage, assist level, pedal cadence
- Power limit from display

**Algorithm**:
```c
power_assist = assist_level × configured_power_factor
target_current = power_assist ÷ battery_voltage
```

### 2. Torque Assist Mode  
**Input Dependencies**:
- Torque sensor delta, torque assist factor
- Pedal cadence (must be > 0)

**Algorithm**:
```c
target_current = torque_delta × torque_assist_factor ÷ denominator
```

### 3. Cadence Assist Mode
**Input Dependencies**:
- Pedal cadence, cadence parameters
- Base torque sensor input

**Algorithm**:
```c
cadence_factor = f(pedal_cadence, cadence_parameters)
target_current = base_torque × cadence_factor
```

### 4. Hybrid Mode
**Input Dependencies**:
- Both power and torque inputs
- Hybrid mixing parameter

**Algorithm**:
```c
power_component = calculate_power_assist()
torque_component = calculate_torque_assist()  
target_current = max(power_component, torque_component)
```

## Safety and Limiting Systems

### Current Limiting
1. **Battery Current Limit**: `ui8_adc_battery_current_max`
2. **Motor Phase Current Limit**: `ui16_adc_motor_phase_current_max`  
3. **Power-based Limit**: Calculated from display power limit

### Protection Systems
- **Undervoltage Protection**: `ui16_adc_voltage < ui16_adc_voltage_cut_off`
- **Overspeed Protection**: Motor ERPS monitoring
- **Brake Cut-off**: Immediate power reduction on brake activation
- **Temperature Protection**: Motor temperature monitoring (if sensor present)

### Ramping Control
```c
// Duty cycle ramping prevents sudden power changes
if (target > current) {
    // Ramp up with ui8_controller_duty_cycle_ramp_up_inverse_step
} else {
    // Ramp down with ui8_controller_duty_cycle_ramp_down_inverse_step  
}
```

## PWM and Motor Drive

### Three-Phase Generation
The system generates three-phase AC from DC using Space Vector Modulation (SVM):

```c
// Phase calculations based on motor position and duty cycle
ui16_a = MIDDLE_SVM_TABLE ± ((table_value - MIDDLE) × duty_cycle >> 8)
ui16_b = MIDDLE_SVM_TABLE ± ((table_value - MIDDLE) × duty_cycle >> 8)  
ui16_c = MIDDLE_SVM_TABLE ± ((table_value - MIDDLE) × duty_cycle >> 8)
```

### FOC Implementation
- **Purpose**: Maintains optimal torque-to-current ratio across all speeds
- **Input**: Hall sensor position, motor speed
- **Output**: Phase angle offset for maximum efficiency
- **Benefits**: Reduced heating, improved efficiency, smoother operation

## Key Constants and Ranges

| Parameter | Range | Default | Units |
|-----------|--------|---------|-------|
| `ui8_g_duty_cycle` | 0-255 | 0 | PWM duty cycle |
| `ui16_a,b,c` | 0-1680 | 840 | PWM counter values |
| `ui8_g_foc_angle` | 0-255 | varies | FOC angle |
| Battery current | 0-143 | varies | ADC steps (23A max) |
| Motor phase current | 0-187 | varies | ADC steps (30A max) |
| PWM frequency | ~19kHz | fixed | Switching frequency |

## Communication with Display

### Transmitted Data (Motor → Display)
- Battery voltage and current
- Motor speed and power
- Torque sensor readings  
- FOC angle and system state
- Hall sensor state
- Temperature probe data (exploration)

### Received Commands (Display → Motor)
- Power limits and assist parameters
- Riding mode selection
- System configuration changes
- Safety limit overrides

## Real-Time Constraints

### Timing Requirements
- **Motor ISR**: Must complete within ~50μs (19kHz rate)
- **ADC Sampling**: Synchronized with PWM switching
- **Main Control**: 25ms update rate sufficient for human response
- **Communication**: UART packets every 100-200ms

### Critical Path Analysis
1. **Hall sensor read** → Motor position (≤5μs)
2. **SVM calculation** → Phase values (≤15μs)  
3. **PWM update** → Hardware registers (≤2μs)
4. **Safety checks** → Immediate shutdown capability (≤1μs)

## Optimization Notes

### Performance Optimizations
- Critical functions marked `__RAM_FUNC` for fast execution
- ADC values pre-scaled to avoid division in ISR
- Lookup tables for trigonometric functions
- Shadow registers for glitch-free PWM updates

### Efficiency Considerations
- FOC control optimizes motor efficiency
- Field weakening extends speed range  
- Current limiting prevents waste heat
- Ramp control provides smooth power delivery

## Diagnostic and Monitoring

### Real-time Monitoring (via UART)
- All input sensor values
- Motor control parameters
- Safety system status
- Performance metrics

### Debug Capabilities
- uCProbe integration for live monitoring
- Oscilloscope outputs for waveform analysis
- Configurable test modes
- Temperature sensor exploration mode

---

*This analysis covers the core motor control system of the TSDZ8 ebike controller. The system demonstrates sophisticated multi-layer control with real-time safety monitoring and efficient motor drive algorithms.*