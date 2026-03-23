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
 * @file PIDController.hpp
 * @brief Header file for a PID controller with setpoint prefiltering and
 * derivative filtering.
 */

#include <chrono>

namespace pid_controller {

struct Options {
    double kp = 1.0;
    double ki = 0.0;
    double kd = 0.0;
    double p_setpoint_weight = 1.0;
    double d_setpoint_weight = 1.0;
    bool use_derivative_filter = false;
    double derivative_cutoff_frequency_hz = 0.0;
    bool integral_cap_enabled = false;
    double integral_cap = 0.0;
    bool use_setpoint_prefilter = false;
    double setpoint_prefilter_cutoff_freq_hz = 10.0;
};

class PIDController {
   public:
    explicit PIDController(const Options& opts = Options{});
    ~PIDController() = default;

    double compute_input(double setpoint, double measured_value,
                         double dt = -1.0);

   private:
    Options options_;
    double prev_error_;
    double prev_d_error_;
    double setpoint_filtered_;
    double derivative_filtered_;
    double integral_;
    bool first_run_;
    double derivative_cutoff_freq_rad_s_;
    double setpoint_prefilter_cutoff_freq_rad_s_;
    std::chrono::steady_clock::time_point last_time_;
};

}  // namespace pid_controller