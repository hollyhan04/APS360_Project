# Fault Type Classification on the IEEE 9-Bus System

APS360 project: classify transmission-line fault types from short three-phase voltage and current
waveforms with a 1D CNN, compared against a simple RNN baseline.

This repository generates the dataset. MATLAB/Simulink simulates faults on the IEEE 9-bus system and
writes labeled waveforms. Training (PyTorch) is separate. Decisions and reasons are in
[PROJECT_LOG.md](PROJECT_LOG.md).

## Requirements

- MATLAB R2023b (other recent releases are untested)
- Simulink, Simscape, and Simscape Electrical

## Quick start

1. Open the example and copy `IEEE9BusSystem.slx` into this folder:

   ```matlab
   openExample("simscapeelectrical/IEEE9BusSystemExample")
   ```

2. Build the fault model and run the sweep:

   ```matlab
   buildFaultModel("IEEE9BusSystem.slx", "IEEE9BusFaults.slx")
   runFaultSweep(22)      % smoke test, a few minutes
   runFaultSweep(3300)    % full dataset: 300 cases per class, about 7 hours
   ```

   Give the smoke test its own folder, for example `OutputFolder="dataset_smoke"`.

## Files

| File | Purpose |
|---|---|
| `buildFaultModel.m` | Copies the 9-bus example into `IEEE9BusFaults.slx` with a fault site on each line |
| `runFaultSweep.m` | Plans the cases, simulates them, and writes the dataset |
| `PROJECT_LOG.md` | Decisions made so far, and why |

`buildFaultModel` does not modify the original example. Each of the six 230 kV lines becomes a
subsystem in the same place. Inside it, the line is split and a four-resistor fault (phases a, b, c
and ground) plus a voltage and current relay are attached. See `help runFaultSweep` for options.

## What each case varies

| Setting | Default |
|---|---|
| Fault class | AG, BG, CG, AB, BC, CA, ABG, BCG, CAG, ABC, NF, equal counts |
| Faulted line | One of L45, L46, L57, L69, L78, L89 |
| Fault position | 10%, 30%, 50%, 70%, 90% from the sending bus |
| Load level | 0.85, 0.925, 1.0, 1.075, 1.15 × nominal |
| Inception angle | One of 64 points around the cycle |
| Fault resistance | 0.01–20 Ω, log-uniform (blank for NF) |
| Ground resistance | 0.01–300 Ω, log-uniform (blank unless the fault includes ground) |

Fault type, resistance, and start time change from case to case without recompiling. Fault position
and load level are compile-time, so the sweep groups cases by those two and recompiles once per group.

## Dataset

`runFaultSweep` writes to `dataset/` (about 80 MB for 3,300 cases; not stored in this repository):

| File | Contents |
|---|---|
| `cases.csv` | Full plan, written once and reused if you resume |
| `sweep_settings.json` | Settings that plan was drawn with |
| `index.csv` | One row per finished case: label, settings, and waveform file |
| `failures.csv` | Cases that errored, only if any |
| `waveforms/case_NNNNNN.csv` | 192 samples: `time_s`, `Va Vb Vc` (V), `Ia Ib Ic` (A) |

`time_s = 0` is fault inception. The window is 1 cycle before and 2 cycles after, taken from the
relay at the sending end of the faulted line. Train from `index.csv`. Waveform columns are the model
inputs; `label` is the target. The other columns are metadata.

## Running the sweep

About 7 s per case, plus about 45 s each time a group recompiles. Stop with Ctrl+C, or by creating an
empty `STOP` file in the output folder. Run the same command to resume; finished cases are skipped.
Do not move the output folder while a sweep is running. A folder planned with different settings is
rejected; use a new `OutputFolder`.
