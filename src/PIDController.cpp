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

/**
 * @file PIDController.cpp
 * @brief Implementation of a PID controller with setpoint prefiltering and
 * derivative filtering.
 *
 * This PID implementation follows standard control design principles:
 *  - Proportional (P), Integral (I), and Derivative (D) terms
 *  - Derivative action with prefiltered setpoint to reduce derivative kick
 *  - Optional low-pass filtering on the derivative to reduce noise
 *
 * Reference:
 * W.S. Levine (Ed.), The Control Handbook, 2nd Edition, CRC Press, 2018.
 * See chapters on PID control and reference/setpoint filtering.
 *
 * Author: Pedro Trindade
 * Date: 2026-03-22
 */

#include "pid_controller/PIDController.hpp"

#include <algorithm>

namespace {
constexpr double PI = 3.14159265358979323846;
}

namespace pid_controller {

PIDController::PIDController(const Options& opts)
    : options_(opts),
      prev_error_(0.0),
      prev_d_error_(0.0),
      setpoint_filtered_(0.0),
      derivative_filtered_(0.0),
      integral_(0.0),
      first_run_(true),
      last_time_(std::chrono::steady_clock::now()) {
    derivative_cutoff_freq_rad_s_ =
        2.0 * PI * options_.derivative_cutoff_frequency_hz;
    setpoint_prefilter_cutoff_freq_rad_s_ =
        2.0 * PI * options_.setpoint_prefilter_cutoff_freq_hz;
}

double PIDController::compute_input(double setpoint, double measured_value,
                                    double dt) {
    auto now = std::chrono::steady_clock::now();

    // On the first run, employ only the proportional term to avoid large
    // initial integral and derivative actions
    if (first_run_) {
        last_time_ = now;
        setpoint_filtered_ = setpoint;
        prev_error_ = setpoint - measured_value;
        prev_d_error_ =
            options_.d_setpoint_weight * setpoint_filtered_ - measured_value;
        first_run_ = false;
        return options_.kp * options_.p_setpoint_weight * setpoint -
               measured_value;
    }

    // Compute errors with setpoint weighting
    double p_error = options_.p_setpoint_weight * setpoint - measured_value;
    double error = setpoint - measured_value;
    double d_error =
        options_.d_setpoint_weight * setpoint_filtered_ - measured_value;

    // If dt is negative, use internal time measurement
    if (dt < 0.0) {
        dt = std::chrono::duration<double>(now - last_time_).count();
    }
    last_time_ = now;

    // Simple first-order low-pass filter for setpoint
    if (options_.use_setpoint_prefilter &&
        setpoint_prefilter_cutoff_freq_rad_s_ > 0.0) {
        double alpha = 1.0 / (1.0 + setpoint_prefilter_cutoff_freq_rad_s_ * dt);
        setpoint_filtered_ =
            alpha * setpoint_filtered_ + (1.0 - alpha) * setpoint;
    } else {
        setpoint_filtered_ = setpoint;
    }

    // Compute integral using the trapezoidal rule.
    // Cap the integral term if enabled.
    integral_ += 0.5 * (error + prev_error_) * dt;
    if (options_.integral_cap_enabled && options_.integral_cap > 0.0) {
        integral_ = std::clamp(integral_, -options_.integral_cap,
                               options_.integral_cap);
    }
    prev_error_ = error;

    // Compute derivative using the backward difference method
    // Apply low-pass filter to the derivative term if enabled
    double derivative = (d_error - prev_d_error_) / dt;
    if (options_.use_derivative_filter && derivative_cutoff_freq_rad_s_ > 0.0) {
        double alpha = 1.0 / (1.0 + derivative_cutoff_freq_rad_s_ * dt);
        derivative_filtered_ =
            alpha * derivative_filtered_ + (1.0 - alpha) * derivative;
    } else {
        derivative_filtered_ = derivative;
    }
    prev_d_error_ = d_error;

    return options_.kp * p_error + options_.ki * integral_ +
           options_.kd * derivative_filtered_;
}

}  // namespace pid_controller
