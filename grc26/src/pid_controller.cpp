#include "grc26/pid_controller.hpp"
#include <cmath>

PID::PID(double p_gain,
         double i_gain,
         double d_gain,
         double error_sum_tol,
         double decay_rate)
{
    err_integ        = 0.0;
    err_last         = 0.0;
    kp               = p_gain;
    ki               = i_gain;
    kd               = d_gain;
    err_sum_tol      = error_sum_tol;
    this->decay_rate = decay_rate;
}

void PID::set_gains(double p_gain,
                    double i_gain,
                    double d_gain,
                    double error_sum_tol,
                    double decay_rate)
{
    err_integ        = 0.0;
    err_last         = 0.0;
    kp               = p_gain;
    ki               = i_gain;
    kd               = d_gain;
    err_sum_tol      = error_sum_tol;
    this->decay_rate = decay_rate;
}

double PID::control(double error, double dt)
{
    if (dt <= 0.0) {dt = 1.0;}  // safety fallback

    // Derivative term
    double err_diff = (error - err_last) / dt;

    if (std::fabs(error) > 0.0) {
        // Accumulate integral when error is non-zero
        err_integ += error * dt;

        // Clamp integral to prevent windup
        if (err_integ > err_sum_tol) {
            err_integ = err_sum_tol;
        } else if (err_integ < -err_sum_tol) {
            err_integ = -err_sum_tol;
        }
    } else {
        // Decay integral when error is zero
        err_integ = decay_rate * err_integ + (1.0 - decay_rate) * error;
    }

    err_last = error;

    return kp * error + ki * err_integ + kd * err_diff;
}