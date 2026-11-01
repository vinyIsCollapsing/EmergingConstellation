# EmergingConstellation

A small swarm of BBC micro:bit v2 boards that synchronize their blinking through radio pulses, without any central coordinator. Each board follows a simple local rule, and the group-level pattern emerges from their interactions.

> **Status:** early stage. The current firmware is a hello-world test to validate the toolchain (Arduino framework, PlatformIO, serial output, LED control).

## Concept

Each board is a pulse-coupled oscillator:

1. Its internal phase advances over time.
2. When the phase reaches its limit, the board flashes and broadcasts a pulse over the 2.4 GHz radio.
3. When a board receives a pulse from a neighbor, it nudges its own phase forward, moving it closer to firing.

No board knows the global state, yet the group tends to converge to a shared rhythm. This model follows Mirollo and Strogatz (1990), inspired by the synchronization of fireflies.

## Roadmap

- [X] **Phase 0:** Hello world on a single board (Arduino framework, serial output, LED blink)
- [ ] **Phase 1:** Two boards exchanging a pulse over radio
- [ ] **Phase 2:** Neighbor discovery and a neighbor table with timestamps
- [ ] **Phase 3:** Pulse-coupled oscillator and first synchronization
- [ ] **Phase 4:** Measurement of synchronization time versus number of nodes, distance, and radio power
- [ ] **Phase 5:** Exploration of other behaviors (waves, clusters, repulsive coupling, Kuramoto model)

## Hardware

- BBC micro:bit v2 (one per node; 5 or 6 nodes planned)
- USB cable for flashing and serial monitoring
- A USB power bank or battery pack for untethered operation (later phases)

## Software

- [PlatformIO](https://platformio.org/) with VS Code
- Platform: `nordicnrf52`
- Board: `bbcmicrobit2`
- Framework: `arduino`

## Getting Started

Clone the repository and build the firmware:

```bash
git clone https://github.com/vinyIsCollapsing/EmergingConstellation
cd EmergingConstellation
pio run
```

Flash the board and open the serial monitor:

```bash
pio run -t upload
pio device monitor
```

Expected output:

```
Hello, EmergingConstellation!
LED on
LED off
```

## Project Structure

```
EmergingConstellation/
├── platformio.ini    # Build configuration (board, platform, framework)
├── src/
│   └── main.cpp      # Firmware entry point
├── include/          # Project header files
├── lib/              # Project-specific libraries
└── test/             # Test code
```

## Design Notes

- **Timing:** the oscillator must not block the loop, so the final version will use `millis()` instead of `delay()`.
- **Radio:** the Arduino core does not expose the micro:bit's proprietary radio API directly. Depending on results, the project may use BLE advertising for pulse broadcast or a lower-level driver.
- **Clock drift:** boards have slightly different clock frequencies. This is part of the observed behavior and will be documented in the measurements.

## License

To be defined.