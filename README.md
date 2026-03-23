# PID-Controller

A modern C++ implementation of a PID (Proportional-Integral-Derivative) controller with advanced features including setpoint prefiltering, derivative filtering, and integral capping.

## Overview

A PID controller is a feedback control mechanism widely used in industrial automation and robotics. This implementation provides a clean and flexible API for integrating PID control into your applications, with support for advanced tuning features such as setpoint weighting, derivative filtering, and optional setpoint prefiltering. The controller is implemented in the **parallel form**, with the implementation following standard control theory as described in:

> 📘 **W. S. Levine (Ed.)**  
> *The Control Handbook, Second Edition*  
> CRC Press, 2018  
>
> Relevant topics: PID control design, derivative filtering, and reference (setpoint) prefiltering


## Features

- **PID Control**: Standard proportional, integral, and derivative terms.
- **Setpoint Prefiltering**: Smooth setpoint transitions with configurable cutoff frequency.
- **Derivative Filtering**: Low-pass filtering on the derivative term to reduce noise sensitivity.
- **Integral Capping**: Prevent integral windup with optional saturation limits.
- **Setpoint Weighting**: Independent control over P and D term response to setpoint changes.
- **Automatic Time Step Handling**: Automatic dt calculation if not provided.

## Building

### Prerequisites

- C++17 or later
- CMake 3.10 or later
- A C++ compiler (g++, clang, MSVC, etc.)

### Build Instructions

```bash
# Create a build directory
mkdir build
cd build

# Configure the project
cmake ..

# Build the library and examples
make
```

## Usage

### Basic Example

```cpp
#include "pid_controller/PIDController.hpp"

// Create controller with default options
pid_controller::PIDController pid;

// In your control loop
double setpoint = 10.0;           // Desired value
double measured_value = 5.0;      // Current measurement
double dt = 0.01;                 // Time step (seconds)

// Compute control input
double control_input = pid.compute_input(setpoint, measured_value, dt);
```

### Advanced Configuration

```cpp
pid_controller::Options opts;
opts.kp = 2.0;                                  // Proportional gain
opts.ki = 0.5;                                  // Integral gain
opts.kd = 0.1;                                  // Derivative gain
opts.p_setpoint_weight = 1.0;                   // P term setpoint weight
opts.d_setpoint_weight = 0.0;                   // D term setpoint weight
opts.use_derivative_filter = true;              // Enable derivative filtering
opts.derivative_cutoff_frequency_hz = 10.0;     // Derivative filter cutoff
opts.integral_cap_enabled = true;               // Enable integral capping
opts.integral_cap = 5.0;                        // Integral saturation limit
opts.use_setpoint_prefilter = true;             // Enable setpoint prefiltering
opts.setpoint_prefilter_cutoff_freq_hz = 2.0;   // Setpoint prefilter cutoff

pid_controller::PIDController pid(opts);
```
#### Fields in the `Options` struct:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `kp` | double | 1.0 | Proportional gain |
| `ki` | double | 0.0 | Integral gain |
| `kd` | double | 0.0 | Derivative gain |
| `p_setpoint_weight` | double | 1.0 | Proportional term setpoint weight |
| `d_setpoint_weight` | double | 1.0 | Derivative term setpoint weight |
| `use_derivative_filter` | bool | false | Enable low-pass filtering on derivative term |
| `derivative_cutoff_frequency_hz` | double | 0.0 | Derivative filter cutoff frequency (Hz) |
| `integral_cap_enabled` | bool | false | Enable integral saturation |
| `integral_cap` | double | 0.0 | Maximum integral term magnitude |
| `use_setpoint_prefilter` | bool | false | Enable setpoint prefiltering |
| `setpoint_prefilter_cutoff_freq_hz` | double | 10.0 | Setpoint prefilter cutoff (Hz) |


## Project Structure

```
.
├── include/
│   └── pid_controller/
│       └── PIDController.hpp      # Main PID controller interface
├── src/
│   └── PIDController.cpp          # Implementation
├── examples/
│   └── example.cpp                # Simulation example
├── CMakeLists.txt                 # Build configuration
└── README.md                      # This file
```

## Running the Example

The example program runs a simulation of the PID controller and outputs results to a CSV file:

```bash
# From build directory
./example

# This creates sim_output.csv with simulation data
```

## License

MIT License - Copyright (c) 2026 Pedro Trindade

