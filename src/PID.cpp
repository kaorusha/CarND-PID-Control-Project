#include "PID.h"

#include <iostream>
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  i_error = 0;
  p_error = 0;
  // twiddle init
  rss = 0;
  loop = 0;
  std::cout << "Update PID: " << Kp << ", " << Ki << ", " << Kd << std::endl;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  i_error += cte;
  d_error = cte - p_error;
  p_error = cte;
  // update twiddle error
  rss += cte * cte;
  loop += 1;
  if (loop > 4) Twiddle(index);
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return Kp * p_error + Ki * i_error +
         Kd * d_error;  // TODO: Add your total error calc here!
}

void PID::Twiddle(int& i) {
  // initialize
  if (!is_init) {
    best_err = rss / loop;
    p = {Kp, Ki, Kd};
    dp = {Kp/10, Ki/10, Kd/10};
    index = 0;
    new_run = true;
    fall_back = false;
    it = 0;
    is_init = true;
  }
  // determine which parameter to update
  i = i % 3;
  if (i == 0) {
    it += 1;
    std::cout << "Iteration " << it << ", best error = " << best_err
              << std::endl;
  }
  if (new_run) {
    p[i] += dp[i];
    new_run = false;
    Init(p[0], p[1], p[2]);
    return;
    // return updating rss until next call
  }
  // calculate error with mean
  double err = rss / loop;
  if (!fall_back) {
    if (err < best_err) {
      best_err = err;
      dp[i] *= 1.1;
      i += 1;  // check next PID value
      new_run = true;
    } else {
      fall_back = true;
      p[i] -= 2 * dp[i];
      // updating rss until next call
      Init(p[0], p[1], p[2]);
    }
  } else {  // fall_back = true
    if (err < best_err) {
      best_err = err;
      dp[i] *= 1.1;
    } else {
      p[i] += dp[i];
      dp[i] *= 0.9;
    }
    i += 1;
    new_run = true;
    fall_back = false;
  }
}