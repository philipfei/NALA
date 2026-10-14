# UART bench test

Test tools for the MCU UART link. They are not robot firmware.

- `mcu_uart_tx_test/`: Studio project for the MCU. It sends a fixed v2.1.0 state frame (`A5 02 ...`) on UART0 (PD1) every 50 ms, 38400 8N1.
  `seq` counts up, `flags` shows the reset cause (bits 0-3) and the clock failure flag (bit 7).
  Flashing it replaces the motor firmware: flash `motor_driver_C` again afterwards.
- `arduino_uart_test.ino`: Arduino Uno sketch. Listens on D13 and D12 in turn, decodes the frames, counts CRC errors and lost frames. Serial Monitor 115200.
  A USB-serial adapter (CH340) on the PC works too: read at 38400 and check the CRC.

Before connecting anything, read the UART wiring note in [`../README.md`](../README.md).
