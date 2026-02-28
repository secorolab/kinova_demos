#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

class PID {
  public:
    PID() = default;
    PID(double p_gain,
        double i_gain,
        double d_gain,
        double error_sum_tol = 1.0,
        double decay_rate    = 0.0);

    void set_gains(
        double p_gain,
        double i_gain,
        double d_gain,
        double error_sum_tol = 1.0,
        double decay_rate    = 0.0
    );

    double control(double error, double dt = 1.0);

  public:
    double err_integ;
    double err_last;
    double kp;
    double ki;
    double kd;
    double err_sum_tol;
    double decay_rate;
};

#endif // PID_CONTROLLER_HPP