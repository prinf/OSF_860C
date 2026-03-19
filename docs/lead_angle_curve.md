# Lead Angle Base Curve — TSDZ8

## Curve Formula

```
angle(RPM) = 0.464 × arctan(0.000297 × RPM)
```

Where:
- RPM = mechanical motor RPM
- angle = lead angle in degrees (electrical)
- 0.464 = scaling factor (accounts for saturation, non-ideal commutation)
- 0.000297 = effective ωL/R per RPM (fitted from empirical data)

## Effective Motor Parameters (from curve fit)

| Parameter | Fitted value | Raw estimate (mstrens) |
|-----------|-------------|----------------------|
| L effective | 163 µH | 150 µH |
| R effective | 230 mΩ | 100 mΩ |
| Pole pairs | 4 | 4 |

The higher effective R (230 vs 100 mΩ) absorbs saturation effects and the fact that
this is sinusoidal commutation, not true FOC.

## Derived Table Values

```c
const uint16_t speed_tab[] = {0,     500,  1000, 2000, 3000, 4000, 4700, 5500};
const float lead_base_deg[] = {0.0f, 4.0f, 8.0f, 14.0f, 19.0f, 23.0f, 25.0f, 27.0f};
```

## Curve Visualization

```
  angle(RPM) = 0.464 × arctan(0.000297 × RPM)

   30.0° |
   28.5° |                                                           ·
   27.0° |                                                    ···●···
   25.5° |                                              ·●····
   24.0° |                                         ·····
   22.5° |                                     ···●
   21.0° |                                 ····
   19.5° |                             ·●··
   18.0° |                          ···
   16.5° |                       ···
   15.0° |                     ··
   13.5° |                  ··●
   12.0° |                ··
   10.5° |             ···
    9.0° |           ··
    7.5° |         ·●
    6.0° |       ··
    4.5° |     ●·
    3.0° |   ··
    1.5° | ··
    0.0° |●
        +────────────────────────────────────────────────────────────
        0         1000      2000      3000      4000      5000      6000
                              RPM (mechanical)

  ● = table points
```

## Empirical Data Used for Fitting (2026-03-19)

| RPM | Observed base | Observed correction | Empirical optimal | Curve |
|-----|--------------|--------------------|--------------------|-------|
| 500 | 2° | ~0° | 2° | 4° |
| 1000 | 5° | ~+2° | 7° | 8° |
| 2000 | 10° | +5° | 15° | 14° |
| 3000 | 15° | +5° | 20° | 19° |
| 4000 | 18° | +5° | 23° | 23° |
| 4700 | 20° | +5° | 25° | 25° |
| 5500 | — | +5° | ~27° | 27° |

RMS fit error: 0.9°

## How This Table Interacts With Other Components

Total lead angle = min(base_rpm + base_current, 35°) + correction(±7°)

- **base_rpm**: this table (0–27° depending on speed)
- **base_current**: battery_ADC × 17 Q8.8 per step (~0–9° at high current)
- **35° cap**: on base total only (RPM + current)
- **correction**: Id-based integrator, ±7° range, 0.01–0.08° steps at 200Hz
- **Effective total range**: ~-3° to ~42°

## Updating the Curve

If future testing shows the correction consistently settling at +N° or -N°:

1. The curve needs shifting. Adjust by changing the **scale factor** (0.464):
   - Correction consistently positive → increase scale
   - Correction consistently negative → decrease scale

2. If the shift varies with speed, adjust **k** (0.000297):
   - More correction needed at high speed → increase k
   - More correction needed at low speed → decrease k

3. Regenerate table values with:
   ```python
   import math
   SCALE, K = 0.464, 0.000297
   for rpm in [0, 500, 1000, 2000, 3000, 4000, 4700, 5500]:
       print(f"{rpm}: {SCALE * math.degrees(math.atan(K * rpm)):.1f}°")
   ```
