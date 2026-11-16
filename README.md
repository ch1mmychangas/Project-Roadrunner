# Project Roadrunner firmware

This firmware targets an ESP32 with the Arduino framework. It reads separate
50 Hz throttle, steering, and drive-mode receiver channels, then generates
50 Hz PPM signals for the front and rear VESC inputs and a PWM signal for the
steering servo. The configured pins are in `include/pinout.h`.

The mode channel is configured as AWD at 1000-1300 us, front dig at
1301-1650 us, and rear dig at 1651-2000 us. The throttle is centered at
1500 us; values above the 25 us deadband request forward drive and values
below it request braking. Steering input from 1000-2000 us maps to the
1150-1850 us servo range.

The vehicle remains disarmed at startup for two seconds, then requires one
continuous second of valid, fresh, neutral-throttle receiver input. Any
receiver channel that is invalid or older than 100 ms forces neutral outputs
and requires the arming sequence again.

## VESC setup and safety checks

The ESP32 only sends PPM pulse widths; it does not communicate over CAN and
cannot enforce VESC current, ERPM, or battery-voltage limits. Configure and
verify the VESC settings independently: Safe Start enabled, 250 ms receiver
timeout with the intended 15 A active-braking behavior, 19.5 A battery-current
limit per channel, +60 A/-40 A motor phase-current limits, and the requested
forward/reverse ERPM limits (50,600/25,000 ERPM). The ESP32 cannot enforce
these settings over PPM.

The drivetrain uses pulses below 1500 us to request braking. Configure the
VESC PPM input so that this range brakes rather than commands reverse, and
bench-test the exact VESC PPM settings before connecting the drivetrain to
the ground. Confirm the VESC accepts 1000-2000 us with a 1500 us neutral.
Ensure the receiver output voltage is safe for ESP32 GPIOs (3.3 V maximum) and
that receiver, ESP32, VESC signal, and servo grounds are connected as required.
Power the high-torque steering servo from an appropriate external supply, not
the ESP32 regulator. Use a raised, unloaded vehicle for initial testing; do
not rely on firmware failsafes as a substitute for a physical emergency stop.

Build with PlatformIO using `pio run -e esp32dev`. The `esp32dev_test`
environment compiles the Unity tests; running them requires a connected ESP32.
