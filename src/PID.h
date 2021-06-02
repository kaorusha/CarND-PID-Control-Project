#ifndef PID_H
#define PID_H
#include <vector>

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();
  /**
   * @brief overloading error calculation with given PID for Twiddle()
   *
   * @param p
   * @param i
   * @param d
   * @return double error
   */
  double TotalError(double p, double i, double d);
  void Twiddle(int& i);

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;

  /**
   * PID Coefficients
   */
  double Kp;
  double Ki;
  double Kd;
  // twiddle coefficients
  std::vector<double> dp;
  std::vector<double> p;
  bool is_init = false; // flag for initializing best_err
  double best_err;
  double rss;  // residual sum square
  int loop;
  int index;
  bool new_run;  // update next PID parameter
  bool fall_back;
  int it;
};

#endif  // PID_H