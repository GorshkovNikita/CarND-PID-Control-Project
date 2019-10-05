#include "PID.h"
#include "helpers.h"
#include <vector>

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;
    p_error = 1.0;
    i_error = 1.0;
    d_error = 1.0;
    prev_cte = 0.0;
}

double PID::CalculateSteering(double cte) {
    double _steering;
    std::vector<double> coef { Kp, Ki, Kd };
    double cte_diff = cte - prev_cte;
    _steering = - Kp * cte - Ki * cte_acc - Kd * cte_diff;
    prev_cte = cte;
    cte_acc += cte;
    return _steering;
}

double PID::TotalError() {
    return p_error + i_error + d_error;
}