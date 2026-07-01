# Active-Noise-Cancellation-through-Kalman-filer
# Hardware-Software Co-Design of Kalman Filters for Active Noise Control

## Project Overview
This project presents a comprehensive hardware-software approach to designing an advanced Active Noise Control (ANC) system. While traditional adaptive filters like Least Mean Squares (LMS) and Recursive Least Squares (RLS) are widely used for acoustic noise cancellation, they frequently suffer from slow tracking and performance degradation when the physical acoustic path changes abruptly. 

To overcome these limitations, this project implements an optimal state-space Kalman estimator to model the primary and secondary noise paths. After validating the mathematical advantages of the Kalman filter via behavioral simulation, the floating-point algorithm was translated into a highly optimized, fixed-point Register-Transfer Level (RTL) architecture in Verilog.

---

## Technical Specifications & Features
* **Algorithmic Benchmarking:** Implemented complete ANC simulation loops for LMS, RLS, and Kalman filters to evaluate convergence speed, steady-state noise reduction, and tracking tracking performance.
* **Fixed-Point Precision:** Discarded resource-heavy floating-point operations in favor of a **Q15.16 fixed-point format** in RTL, balancing logic area constraints with the strict dynamic range requirements of matrix algebra.
* **Time-Multiplexed Hardware Datapath:** Implemented an area-efficient micro-architecture featuring a single, high-speed Multiply-Accumulate (MAC) unit shared across all execution states to avoid the massive hardware overhead of parallel multipliers.
* **Memory Subsystem:** Inferred structural dual-port Block RAMs (BRAM) to store the 32-tap reference input buffer, system state vectors, and the $32 \times 32$ error covariance matrix.
* **Multi-Cycle Execution Control:** Designed a robust control path controlled by a Finite State Machine (FSM) that orchestrates memory addressing, MAC accumulation loops, and handshakes with a multi-cycle hardware divider.

---

## Repository Structure
* **matlab/**
    * `simulation.m`: Baseline simulation scripts comparing convergence curves and steady-state misalignment across LMS, RLS, and Kalman variants.
    * `simulation_shock.m`: Evaluation script modeling transient response and recovery behavior following an instantaneous change in the acoustic path.
* **verilog/**
    * `kalman_anc_core.v`: Synthesizable top-level Verilog core integrating the control FSM, memory management logic, fixed-point MAC engine, and divider interfaces.
    * `tb_kalman.v`: Testbench file feeding discrete impulse data into the core to verify clock-cycle accurate state transitions and output generation.

---

## Performance Evaluation & Results

### 1. Software Simulation Metrics (MATLAB)
All three algorithms were evaluated over 6,000 iterations using a 32-tap FIR filter framework. The measurement noise floor was modeled at a variance of 0.1, bounding the optimal target noise reduction at approximately **-20 dB**.

* **Initial Convergence Speed:** The Kalman filter reached the critical **-15 dB** noise reduction threshold within **101 iterations**, outperforming the LMS filter which required **216 iterations**.
* **System Identification Accuracy:** The Kalman approach achieved a steady-state normalized misalignment of **0.0696**, indicating highly precise tracking of the unknown true plant.
* **Transient Path-Shock Tracking:** At iteration 3000, an abrupt acoustic path shift was introduced (a 3-sample delay insertion). The Kalman filter rapidly adapted its internal gain distribution, recovering back down to the -15 dB threshold in just **124 iterations**. In comparison, LMS required **294 iterations**, while RLS suffered from severe covariance stalling, requiring **1539 iterations** to recover.

### 2. RTL Verification (Icarus Verilog & GTKWave)
The structural integrity of the digital design was verified through cycle-accurate simulation:
* The control FSM correctly transitions from `IDLE` through `SHIFT_U`, `FILTER_OUT`, `DENOM_CALC`, `DIVIDE`, `GAIN_UPDATE`, `STATE_UPDATE`, and `COV_UPDATE` upon receiving a `start` pulse.
* Data synchronization is strictly maintained: the a priori error vector (`a_priori_err`) and the filter output (`y_out`) lock smoothly at the transition boundary between the filter execution and denominator calculation states.
* The system asserts a single-cycle `done` flag exactly upon completing the final row and column iterations of the covariance matrix update, clearing the internal datapath for the next incoming audio sample.

---

## Setup and Execution

### Tools Required
* MATLAB (R2022a or later recommended)
* Icarus Verilog compiler (`iverilog`)
* GTKWave Waveform Viewer

### Running the Hardware Simulation
To compile the Verilog source code and analyze the execution waveforms, run the following commands in your terminal:

```bash
# Clone the repository and navigate to the verilog directory
cd repository-root/verilog

# Compile the top-level core and its corresponding testbench
iverilog -o kalman_sim kalman_anc_core.v tb_kalman.v

# Execute the simulation file to generate the VCD waveform dump
vvp kalman_sim

# Open the compiled wave trace inside GTKWave
gtkwave kalman_waves.vcd
