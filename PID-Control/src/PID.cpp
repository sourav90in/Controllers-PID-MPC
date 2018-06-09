#include "PID.h"

using namespace std;
using namespace pid_n;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  this->Kp = Kp_;
  this->Ki = Ki_;
  this->Kd = Kd_;
  p_error = 0.0;
  i_error = 0.0;
  d_error = 0.0;

}

void PID::UpdateError(double cte) {

  d_error = cte - p_error ;
  i_error += cte ;
  p_error = cte;
}

double PID::TotalError() {
  return ( -Kp*p_error - Kd*d_error - Ki*i_error );
}

void PID::SetHypParams(Twiddle& tw)
{
  this->Kp = tw.params[0];
  this->Ki = tw.params[1];
  this->Kd = tw.params[2];
}

void PID::PrintHypParams()
{
  cout<<Kp<<" "<<Ki<<" "<<Kd<<"\n";
}

void PID::ResetError()
{
  p_error = 0.0;
  d_error = 0.0;
  i_error = 0.0;
}

int Twiddle::HypTuning(int steps, bool is_complete_track)
{
  double sum_p = 0;

  for(int i=0; i < 3; i++)
  {
    sum_p += dp_params[i];
  }

  if( sum_p < 0.002 ) return 0;
  else
  {
    ParamTuner( idx_opt,steps,is_complete_track);
    return -1;
  }
}

void Twiddle::ParamTuner(int idx, int steps,bool is_complete_track)
{
  cout<<"Current idx is:"<<idx<<"\n";
  if(!st1_flag)
  {
    params[idx] += dp_params[idx];
    ResetError();
    st1_flag = true;
    return;
  }

  if( ( cur_error/steps < best_error ) && ( is_complete_track == true ) )
  {
    best_error = cur_error/steps;

    /* Save the best params */
    best_params[0] = params[0];
    best_params[1] = params[1];
    best_params[2] = params[2];

    dp_params[idx]*= 1.1;
    st1_flag = false;
    idx_opt = idx_opt+1;

    if( idx_opt <= 2)
    {
      ParamTuner(idx_opt,steps,is_complete_track);
    }
    else
    {
      idx_opt = 0;
      ResetError();
      return;
    }

  }
  else
  {
    if(!st2_flag)
    {
      params[idx] -= 2*dp_params[idx];
      ResetError();
      st2_flag = true;
      return;
    }

    /* Only treat the average error as valid in order to compare it with the
     * last recorded best error if the car has been able to complete 1150 steps
     * i.e. the complete track, otherwise a pentalty is imposed in the sense that
     * the current error is treated as implicitly worse than the best error
     */
    if( ( cur_error/steps < best_error ) && ( is_complete_track == true ) )
    {
      best_error = cur_error/steps;

      /* Save the best params */
      best_params[0] = params[0];
      best_params[1] = params[1];
      best_params[2] = params[2];
      dp_params[idx] *= 1.1;
    }
    else
    {
      params[idx] += dp_params[idx];
      dp_params[idx] *= 0.9;
    }

    st1_flag = false;
    st2_flag = false;
    idx_opt = idx_opt+1;

    if( idx_opt <= 2)
    {
      ParamTuner(idx_opt,steps,is_complete_track);
    }
    else
    {
      idx_opt = 0;
      ResetError();
      return;
    }
  }

}

void Twiddle::PrintDPParams()
{
  cout<<dp_params[0]<<" "<<dp_params[1]<<" "<<dp_params[2]<<" \n";
}

void Twiddle::PrintBestParams()
{
  cout<<"Best params till now are:";
  cout<<best_params[0]<<" "<<best_params[1]<<" "<<best_params[2]<<" \n";
}
