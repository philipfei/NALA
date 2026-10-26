# Scintillator input (app 3)

The scintillator measures radiation. In app 3 its readings make the heatmap on the robot screen, find
contaminated spots (hotspots), and stop the scan when contamination is found (user, 2026-10-22).

## Device and protocol (user, 2026-10-14 and 2026-10-26)

- Connected to the Pi over **USB**, through a Prolific USB-to-serial adapter (USB id 067b:23a3),
  `/dev/serial/by-id/usb-Prolific_Technology_Inc._USB-Serial_Controller_ESCDb147612-if00-port0`.
- **19200 baud, 8 data bits, no parity, 1 stop bit, hardware flow control (RTS/CTS).** The Pi only reads; it
  sends nothing. (CTS reads low; receiving works anyway.)
- One text line about **every second**, fields separated by TAB, ending in CR LF. Read on the robot 2026-10-26:

  ```
  0 <TAB> <TAB> 21-10-2026 13:57:39 <TAB> 0.000 <TAB> cps <TAB> 100 <TAB> % <TAB> 4 <TAB> cps <TAB> 47 <TAB> %
  ```

- **Only the value before the second `cps` field is used** (here 4 counts/s; user, 2026-10-26). The `%` after it
  looks like the statistical error (about 1/sqrt(counts)). Every other field is ignored: the first `0`, the empty
  field, the scintillator's own date and time (its clock is wrong, and the Pi time of arrival is used) and the first
  cps value (always 0.000 / 100 % so far). A line without a second cps value is skipped (and counted in the log).

## The driver

`nala_base/scintillator_node` (parser `nala_base/scintillator.py`, unit tests `test_scintillator.py`, settings
`config/scintillator.yaml`). App 3 starts it on the robot (`hardware:=true`). It opens the port exclusively: while it
runs, no other program (for example miniterm) can read the scintillator. Live readings in a terminal: README, app 3.

## The interface the rest of app 3 uses

The driver publishes this topic. The simulation (`tools/sim/fake_nala.py`) publishes the same topic.

| Topic | Type | Rate |
|---|---|---|
| `/scintillator/count_rate` | `nala_interfaces/msg/CountRate` | one message per counting interval (about 1 Hz) |

`CountRate` (`src/nala_interfaces/msg/CountRate.msg`):

| Field | Meaning |
|---|---|
| `header.stamp` | Pi time at the **end** of the counting interval (the time the value arrived is good enough) |
| `counts_per_second` | the counts of the interval divided by its length |
| `interval_s` | length of the counting interval in seconds (about 1.0) |

The reading belongs to the time span `[stamp - interval_s, stamp]`.

## What app 3 does with a reading (`nala_coverage/radiation.py`, `radiation_node.py`)

1. **Background:** at the start of every task the robot stands on the base station. The first
   `radiation.background_readings` readings (`config/coverage.yaml`) are averaged: that is the background.
   The robot does not drive before this is done; without readings the task pauses (`NO_SCINTILLATOR`).
2. **Where the reading was taken:** the supervisor publishes the measured robot pose 10 times a second
   (`/coverage/sample`: AMCL + odometry, with a "trusted" flag). A reading is painted on the map cells under the
   sensor point (`geometry.sensor_offset_m` ahead of the robot) of every pose in its time span, and only if all
   those poses are trusted and there is no gap. Otherwise it is written to the log but not drawn. So the map shows
   only where the sensor really was, never where the plan says it should have been.
3. **Heatmap:** every cell keeps the highest reading painted on it.
4. **Threshold:** set by the operator on the robot screen (Maps -> Settings), for every map: either a count rate
   (counts/s) or a factor x the background (above 1). It is saved in `maps/contamination_threshold.yaml` and sent
   latched on `/radiation/threshold` (JSON `{"mode": "cps" | "background", "value": ...}`). Until one is saved,
   `radiation.threshold_mode` / `threshold_cps` / `threshold_factor` in `config/coverage.yaml` are used. The
   background is measured at every start in both modes (step 1).
5. **Hotspots (display):** cells at the threshold or more; cells closer than `hotspot_join_m` are one hotspot
   (number, highest reading, position, first seen, checked yes/no).
6. **Contamination found (the supervisor):** `radiation.alarm_readings` of the last `radiation.alarm_window`
   readings (2 of 3) at the threshold or more, at any time during a task (driving, paused, measuring the
   background in counts/s mode, driving home). Every reading counts, also one taken while the position was not
   trusted (the spot is then marked approximate). The robot stops like a pause (zero speed at once, ~2 cm coasting from 0.12 m/s), and the
   task ends: `FAILED`, reason `CONTAMINATION_FOUND`. No resume, no driving home: the robot stays where it stopped.
   The robot screen shows a full-screen red warning (readings, threshold, time, sensor position) until Acknowledge,
   then the spot on the map. The task report (`task_*.json`, key `contamination`) keeps the details; the Results
   tab shows them. The robot never drives around contamination (the keepout zone of before 2026-10-22 is removed).

Run folder (`maps/<name>/runs/<time>/`): `readings.csv` (every reading: time, counts/s, sensor x, y, drawn yes/no),
`hotspots.json`, `heatmap.npz`.

## Open points

- The counting interval (1.0 s assumed: one line per second) and whether the stamp should be corrected for a delay.
- What the first cps value, the first field and the empty field mean (not needed: ignored).
- The real background and source count rates, and a good threshold (the config values are placeholders). At low
  rates (a few counts per second), Poisson noise alone often reaches 2x background in one reading; the 2-of-3 rule
  makes a false stop less likely, but test it with the real scintillator.
- The sensor position and reach (`sensor_offset_m`, `coverage_disk_radius_m` in `config/coverage.yaml`) are
  still placeholders.

## Without a scintillator

Set `radiation.enabled: false` in `config/coverage.yaml`: no background measurement, no contamination stop.
It is false now (user, 2026-10-26: until the real background and a good threshold are known). The driver still
runs, so the heatmap, hotspots and `readings.csv` are recorded. The simulation
(`tools/sim/fake_nala.py --sources ...`) publishes `CountRate` from simulated sources.
