#ifndef PID_H
#define PID_H
// Implement a PID controller class
class PIDController {
  private:
    float kp_;
    float ki_;
    float kd_;
    int summed_error_limit_;
    int control_limit_min_;
    int control_limit_max_;
    int summederror_;
    int lasterror_;
  
  public:
    PIDController(float kp, float ki, float kd, int summed_error_limit, int control_limit_min, int control_limit_max) {
      kp_ = kp;
      ki_ = ki;
      kd_ = kd;
      summed_error_limit_ = summed_error_limit;
      control_limit_min_ = control_limit_min;
      control_limit_max_ = control_limit_max;
      summederror_ = 0;
      lasterror_ = 0;
    }

    int compute(int target, int current) {
      int error = target - current;
      summederror_ = summederror_ + error;

      // Anti-windup
      if (summederror_ > summed_error_limit_) {
        summederror_ = summed_error_limit_;
      }
      if (summederror_ < -summed_error_limit_) {
        summederror_ = -summed_error_limit_;
      }

      // Basic PID formula
      int output = kp_ * error + ki_ * summederror_ + kd_ * (error - lasterror_);

      // Control limits
      if (output > control_limit_max_) output = control_limit_max_;
      if (output < control_limit_min_) output = control_limit_min_;

      lasterror_ = error;
      return output;
    }
};

#endif