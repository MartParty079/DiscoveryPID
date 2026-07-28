# DiscoveryPID

PI speed-control development for an Arduino Mega 2560, Adafruit 4416 geared encoder motor, and Adafruit DRV8871 motor-driver breakout.

## Active hardware

- Arduino Mega 2560 / ATmega2560
- Adafruit Product 4416 geared DC motor with magnetic quadrature encoder
- Adafruit Product 3190 DRV8871 brushed DC motor-driver breakout
- External motor supply near 7 VDC
- JMP Bayesian Optimization for experimental PI-gain tuning

## Active firmware

The only active PlatformIO application source is:

```text
src/main.cpp
```

Older sketches and hardware tests are stored under:

```text
examples/
```

PlatformIO does not compile the example sketches as part of the active application.

## Wiring

### Encoder motor

| Motor wire | Connection |
|---|---|
| Black | Arduino GND |
| Blue | Arduino 5 V |
| White | Mega pin 2 / encoder A |
| Yellow | Mega pin 3 / encoder B |
| Red and green | DRV8871 OUT1 and OUT2 |

### DRV8871

| Driver connection | Connection |
|---|---|
| IN1 | Mega PWM pin 9 |
| IN2 | Mega PWM pin 10 |
| VM | External motor supply near 7 V |
| GND | Supply ground and Arduino GND |

Do not connect the external motor-supply voltage to the Arduino 5 V pin.

## PlatformIO

The project is configured for:

```ini
[env:megaatmega2560]
platform = atmelavr
board = megaatmega2560
framework = arduino
monitor_speed = 115200
```

Build and upload using the PlatformIO controls in VS Code, or run:

```bash
pio run
pio run --target upload
pio device monitor
```

## Controller use

1. Power up with the motor mechanically safe to rotate.
2. Open the serial monitor at 115200 baud.
3. Send a numeric output-shaft target RPM, such as `100`.
4. Record the printed target, output RPM, motor RPM, error, integral state, encoder counts, and PWM command.

The controller starts at a target of 0 RPM to prevent automatic movement after reset.

## Reference hardware pages

- Adafruit 4416: https://www.adafruit.com/product/4416
- Adafruit DRV8871: https://www.adafruit.com/product/3190
