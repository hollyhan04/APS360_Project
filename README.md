# Fault Type Classification on the IEEE 9-Bus System

APS360 project: classify transmission-line fault types from short three-phase voltage and current
waveforms with a 1D CNN, compared against a simple RNN baseline.

This repository holds the data-generation part. MATLAB/Simulink simulates faults on the IEEE 9-bus
system and writes a labeled waveform dataset. The training code (PyTorch) works on that dataset.

## Requirements

- MATLAB R2023b (other recent releases likely work but are untested)
- Simulink, Simscape, and Simscape Electrical

## Quick start

1. Get the example model and copy `IEEE9BusSystem.slx` into this folder:

   ```matlab
   openExample("simscapeelectrical/IEEE9BusSystemExample")
   ```

2. From this folder, build the fault model and run the sweep:

   ```matlab
   buildFaultModel("IEEE9BusSystem.slx", "IEEE9BusFaults.slx")
   runFaultSweep(22)      % smoke test: two cases per class, a few minutes
   runFaultSweep(3300)    % full dataset: 300 cases per class, about 7 hours
   ```

   Use a different `OutputFolder` for the smoke test, for example
   `runFaultSweep(22, OutputFolder="dataset_smoke")`, so it does not collide with the full run.

## Files

| File | Purpose |
|---|---|
| `buildFaultModel.m` | Converts the 9-bus example into `IEEE9BusFaults.slx` with switchable faults and relays |
| `runFaultSweep.m` | Plans fault cases, simulates them, and writes the dataset |

## The fault model

`buildFaultModel` edits a copy of the example; the original model is not changed.

- **Fault sites:** each of the six 230 kV lines (4–5, 4–6, 5–7, 6–9, 7–8, 8–9) is replaced by a
  subsystem in the same place, labeled with the line name plus `Fault site L45` (and so on). It has
  one connection to each bus, so the top level looks like the original example. Inside, the line is
  split into a near and a far section. The split point carries a fault made of four variable
  resistors: phases a, b, c to a star point, and the star point to ground. Which resistors close sets
  the fault type.
- **Relays:** inside each fault site, a current sensor and a phase-to-ground voltage sensor sit at the
  sending end of the line. Their signals are logged as `i_L45`, `v_L45`, and so on.
- **Fault timing:** one shared signal, `faultActive`, switches from 0 to 1 at `faultStartTime`. It
  reaches every fault site through a Goto/From tag, so no wires cross the diagram.
- **Sample time:** 64 samples per 60 Hz cycle (3840 Hz). The example used one sample per cycle.
- **Load level:** loads A, B, and C are scaled together by one factor.

Fault type, resistance, and start time are Simulink signals, so they change between cases under
Fast Restart without recompiling. Fault position and load level are compile-time values, so the
sweep runs cases grouped by those two settings and recompiles once per group.

## What each case varies

| Setting | Values (defaults) |
|---|---|
| Fault class | AG, BG, CG, AB, BC, CA, ABG, BCG, CAG, ABC, NF (no fault); equal number of cases per class |
| Faulted line | One of the six lines, uniform |
| Fault position | 10%, 30%, 50%, 70%, 90% of the line from the sending bus |
| Load level | 0.85, 0.925, 1.0, 1.075, 1.15 × nominal |
| Inception angle | 0–360°, in steps of 5.625° (one sample) |
| Fault resistance (per phase) | 0.01–20 Ω, log-uniform |
| Ground resistance (grounded faults) | 0.01–300 Ω, log-uniform |

All of these can be changed with name-value options; see `help runFaultSweep`.

