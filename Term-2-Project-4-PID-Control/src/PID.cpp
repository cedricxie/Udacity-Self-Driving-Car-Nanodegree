#include "PID.h"

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;
  
  sum_error = 0;
  prev_error = 0;
  
}

void PID::UpdateError(double cte) {
  
  sum_error += cte;
  
  p_error = -Kp * cte;
  d_error = -Kd*(cte - prev_error);
  i_error = -Ki*sum_error;
  
  prev_error = cte;
  
}

double PID::TotalError() {

  return p_error+d_error+i_error;

}

