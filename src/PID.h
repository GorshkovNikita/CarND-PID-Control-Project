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

  void InitTwiddle();

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  void Twiddle();

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

  int GetIterations();

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
  std::vector<double> p;

  bool twiddling;
  double twiddleTolerance;
  int iterations;
  std::vector<double> dp;
  double totalError;
  double bestError;
  int curIdx;
  int iterationsToIgnore;

  enum TwiddleState {
      STARTED,
      INCREMENT,
      DECREMENT
  };

  TwiddleState state;

};

#endif  // PID_H