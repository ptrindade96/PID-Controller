/*
 * MIT License
 *
 * Copyright (c) 2026 Pedro Trindade
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <fstream>
#include <iostream>
#include <vector>

#include "pid_controller/PIDController.hpp"

// Structure to hold a single simulation sample
struct SimSample {
    double time;
    double setpoint;
    double output;
    double input;
};

int main() {
    // Simulation parameters
    double sim_time = 10.0;  // seconds
    double dt = 0.025;        // time step in seconds

    // Simple SISO system dynamics: x_dot = a*x + b*u, y = c*x
    double a = 1.0;
    double b = 1.0;
    double c = 2.0;

    // Initial conditions
    double x = 0.0;

    // Desired setpoint trajectory
    double y_d = 10.0;

    // Vector to hold simulation data
    std::vector<SimSample> data;

    // Create PID controller with custom options
    pid_controller::Options opts;
    opts.kp = 1.5;
    opts.ki = 1.5;
    opts.kd = 0.2;
    opts.p_setpoint_weight = 1.0;
    opts.d_setpoint_weight = 0.0;
    opts.use_derivative_filter = true;
    opts.derivative_cutoff_frequency_hz = 10.0;
    opts.integral_cap_enabled = true;
    opts.integral_cap = 10.0;

    pid_controller::PIDController pid(opts);

    // Simulation loop
    for (double t = 0; t < sim_time; t += dt) {
        // Change setpoint at t=5s to demonstrate response to setpoint change
        if (t > 5.0) {
            y_d = -5.0;
        }

        // Simulate measurement noise in range [-0.1, 0.1]
        double noise = 0.2 * ((rand() / (double)RAND_MAX)) - 0.1;
        double y = c * x + noise;

        // Compute control input using PID controller
        double u = pid.compute_input(y_d, y, dt);

        // Simulate system dynamics
        x += (a * x + b * u) * dt;

        // Store simulation data
        data.push_back({t, y_d, y, u});
    }

    // Write data to CSV
    std::ofstream file("sim_output.csv");
    file << "time,setpoint,output,input\n";
    for (const auto& sample : data) {
        file << sample.time << "," << sample.setpoint << "," << sample.output
             << "," << sample.input << "\n";
    }
    file.close();

    std::cout << "Simulation complete. Data written to sim_output.csv\n";

    return 0;
}