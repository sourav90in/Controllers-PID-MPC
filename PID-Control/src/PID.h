#ifndef PID_H
#define PID_H

#include <cfloat>
#include <iostream>

namespace pid_n
{
class Twiddle;
class PID;
class Bprop;
enum Tuningalgo
{
  Twiddling = 0,
  BackProp = 1
};
using namespace std;
}

#define TOT_STEPS 1150

class pid_n::PID {
public:
  /*
  * Errors
  */
  double p_error;
  double i_error;
  double d_error;

  /*
  * Coefficients
  */
  double Kp;
  double Ki;
  double Kd;

  /*
   * To keep track of whether its tuning phase or operational phase
   * needs to be Set to true when Twiddling is needed
   */
  bool isTune_Needed{false};

  /* To keep track of setting the best error the first time hyperparamter tuning kicks off */
  bool isfirstTuned{false};

  /* To select the hyper-parameter tuning algorithm*/
  Tuningalgo algo{Twiddling};

  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp_, double Ki_, double Kd_);

  /*
  * Update the PID error variables given cross track error.
  */
  void UpdateError(double cte);

  /*
   * Resets all the error variables
   */
  void ResetError(void);

  /*
  * Calculate the total PID error.
  */
  double TotalError();

  /*
   * Sets the hyperparameters to values
   * obtained by Twiddling
   */
  void SetHypParams(pid_n::Twiddle& tw);

  /* Prints the value of the hyperparameters */
  void PrintHypParams(void);

};

class pid_n::Twiddle
{
private:

  /* This function performs the runing of the ith hyper-parameter indexed by
   * idx
   */
  void ParamTuner(int idx, int steps,bool is_complete_track);

  /* This array holds the values configured for all
  * the 3 hyper-params
  */
 double params[3];

 /* This array holds the best param values obtained till now */
 double best_params[3];

  /* This array holds the differences in params */
  double dp_params[3];

  /* This holds the parameter index being currently optimized*/
  int idx_opt{0};

  /* This holds the status of the first breakout stage of tuning the ith param */
  bool st1_flag{false};

  /* This holds the status of the second breakout stage of tuning the ith param */
  bool st2_flag{false};

  /* This param stores the best error achieved till now */
  double best_error{DBL_MAX};

 public:

  /* This param stores the current error */
  double cur_error{0.0};

  /* Used to set the current error achieved till now */
  void setError(double cte)
  {
    cur_error += cte*cte;
  }

  /* Used to set the best error */
  void setBestError()
  {
    best_error = cur_error/TOT_STEPS;
  }

  /* Used to set the current error achieved till now */
  void ResetError(void)
  {
    cur_error = 0;
  }

  /* Train the PID controller to learn the hyperparamters */
  int HypTuning(int steps, bool is_complete_track);

  /* Prints the best params obtained till now */
  void PrintBestParams();

  /* constructor */
  Twiddle() {}

  /* Destructor */
  ~Twiddle() {}

  /* Initialize */
  void Init(double Kp_, double Ki_, double Kd_)
  {
    params[0] =  Kp_;
    params[1] = Ki_;
    params[2] = Kd_;
    best_params[0] =  Kp_;
    best_params[1] = Ki_;
    best_params[2] = Kd_;

    /* These do_params were chosen based on the variety of values for
     * each paramter that worked during manaul tuning. The idea here was
     * to allow Twiddle converge to values in the neighbourhood of manually
     * determined parameter values. Keeping these too high such would cause Twiddle
     * to take forever.
     */
    dp_params[0] = 0.2;
    dp_params[1] = 0.001;
    dp_params[2]= 0.75;
  }

  /* Prints the value of the hyperparameters */
  void PrintDPParams(void);

  friend class PID;
};
#endif /* PID_H */
