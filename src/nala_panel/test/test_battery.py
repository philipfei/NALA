from nala_panel.battery import BatteryMeter, battery_text


class FakeBus:
    def __init__(self, registers):
        self.registers = list(registers)

    def read_i2c_block_data(self, address, register, length):
        value = self.registers.pop(0)
        if value is None:
            raise OSError('no answer')
        raw = int(round(value / 0.004)) << 3
        return [raw >> 8, raw & 0xFF]


def test_moving_average_of_the_last_samples():
    meter = BatteryMeter(1, 0x40, 3)
    meter.bus = FakeBus([10.0, 16.0, 16.2, 16.4])
    assert meter.voltage() is None
    for _ in range(4):
        meter.read()
    assert abs(meter.voltage() - 16.2) < 1e-6


def test_no_answer_drops_the_readings():
    meter = BatteryMeter(1, 0x40, 3)
    meter.bus = FakeBus([16.0, None])
    meter.read()
    meter.read()
    assert meter.voltage() is None


def test_text_rounds_and_clamps():
    assert battery_text(16.54, 12.0, 16.8) == '16.5 V  95%'
    assert battery_text(17.2, 12.0, 16.8) == '17.2 V  100%'
    assert battery_text(11.0, 12.0, 16.8) == '11.0 V  0%'
    assert battery_text(None, 12.0, 16.8) == '-- V  --%'
