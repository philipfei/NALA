# Battery voltage monitor (INA219)

Status 2026-10-21: works on the robot. `i2cdetect -y 1` shows `40`; `battery_node` alone + `ros2 topic echo
/battery_state --field voltage` printed 16.44 V at 1 Hz (not compared with a multimeter yet). The battery check in app 3 stays off
(`battery.enabled: false` in `config/coverage.yaml`) and the robot screen does not show the battery yet.

## Hardware

- Chip: TI INA219 on the Pi I2C bus 1 (GPIO2 = SDA, GPIO3 = SCL, 3.3 V). I2C address **0x40** (user, 2026-10-21).
- IN+ and IN- are shorted: the chip measures only the battery voltage (bus voltage register). **No current is measured.**
- Battery: 4S2P Li-ion 18650, 16.8 V full (4.2 V/cell), 14.0 V low alarm (3.5 V/cell), 12.0 V empty (3.0 V/cell), 6.6 Ah.
- An old INA260 (dead, never answered) is still soldered on the same I2C lines. It must not pull SDA or SCL low and
  must not use address 0x40. Before the first test: `i2cdetect -y 1` must show `40`.

## Code

| What | Where |
|---|---|
| Register decoding (the file is still called `ina260.py`, the name was kept on purpose) | `src/nala_base/nala_base/ina260.py` |
| ROS node | `src/nala_base/nala_base/battery_node.py` (started by the app 1 and app 3 launch files) |
| Settings (address, voltages, rate) | `config/battery.yaml` |
| Unit tests | `src/nala_base/test/test_ina260.py` |

Bus voltage register 0x02, 2 bytes, big endian. Voltage = (register >> 3) x 4 mV. The two low bits are flags (CNVR, OVF) and
are ignored. The chip starts in the 32 V range, which is enough for 16.8 V. No setup write is needed.

## Data format

The node reads the voltage once per `publish_rate` (1 Hz) and publishes:

`/battery_state` (`sensor_msgs/BatteryState`)

| Field | Value |
|---|---|
| `header.stamp` | Pi time of the reading |
| `voltage` | Battery voltage in V (for example 16.1) |
| `current` | `NaN` (not measured) |
| `charge`, `capacity` | `NaN` |
| `design_capacity` | 6.6 (Ah, from `battery.yaml`) |
| `percentage` | 0.0 .. 1.0 (a fraction, not percent). Linear between `empty_voltage` 12.0 V and `full_voltage` 16.8 V, clamped. Rough: the 18650 curve is not linear and the voltage sags under load |
| `present` | `True` |
| `power_supply_technology` | Li-ion. Status and health are UNKNOWN |

`/battery_low` (`std_msgs/Bool`): `True` while the voltage is below `low_voltage` (14.0 V), otherwise `False`. Published at the
same rate. While low, the node logs a warning at most every 30 s.

If the chip does not answer, the node logs "Cannot read the INA219" (at most every 30 s) and publishes nothing.

## Who uses it

- The app 3 supervisor reads `/battery_state.percentage` only when `battery.enabled` is `true`. Then it needs a message younger than
  `health.battery_age_s` (15 s) and a finite percentage in 0..1 (`BATTERY_STALE`, `BATTERY_INVALID`, `BATTERY_CRITICAL`).
  It does not read `current`, so `NaN` is fine.
- Not done yet: showing the battery on the robot screen; enabling the battery check.
