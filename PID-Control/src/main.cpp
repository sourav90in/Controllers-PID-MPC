#include <uWS/uWS.h>
#include "PID.h"
#include "json.hpp"
#include <math.h>
#include <fstream>

// for convenience
using json = nlohmann::json;
using namespace std;
using namespace pid_n;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
std::string hasData(std::string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != std::string::npos) {
    return "";
  }
  else if (b1 != std::string::npos && b2 != std::string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main(int argc, char *argv[])
{
  uWS::Hub h;

  PID pid;
  Twiddle tw;
  double kp_init;
  double ki_init;
  double kd_init;
  double cte_thresh;
  unsigned long step_counter = 0;
  //This is needed because of time-lag between Reset command being issued and the reset actually happening.
  bool is_reset_pending = false;

  if(argc == 5){
      kp_init = atof(argv[1]);
      ki_init = atof(argv[2]);
      kd_init = atof(argv[3]);
      cte_thresh = atof(argv[4]);
    }
  else
  {
    /* When triggering Twiddling, I had set these values as
     * 0.30, 0.0002 and 3.
     * After twiddling the output values have been set below
     */
    kp_init = 0.389084; //0.30;
    ki_init = 0.001524251; //0.0002;
    kd_init = 3.41323; //3
    cte_thresh = 2.5;
  }

  /* Initializations of Twiddle and PID Instances */
  pid.Init(kp_init, ki_init,kd_init);
  tw.Init(kp_init, ki_init,kd_init);

  h.onMessage([&pid, &tw, &step_counter,&kp_init, &ki_init, &kd_init,&is_reset_pending,cte_thresh](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      auto s = hasData(std::string(data).substr(0, length));
      if (s != "") {
        auto j = json::parse(s);
        std::string event = j[0].get<std::string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          double cte = std::stod(j[1]["cte"].get<std::string>());
          double speed = std::stod(j[1]["speed"].get<std::string>());
          double angle = std::stod(j[1]["steering_angle"].get<std::string>());
          double steer_value = 0.0;

          if( is_reset_pending == false ) {
            /* Needed for the case of internal Simulator Resets
             * while performing hyper-parameter tuning
             */
            if( ( step_counter  == 0 )  && ( pid.isTune_Needed == true )  && (pid.algo == Twiddling ) )
            {
              pid.ResetError();
            }

            pid.UpdateError(cte);
            tw.setError(cte);
            //Increment the time-step counter
            step_counter++;

            /* This is needed because while auto-tuning, some choice of parameters
             * were causing the car to drive off the track an hence were definitely not
             * optimal. the cte_thresh was determined empirically as a value fo 4 to stay within
             * the track premises. In cases where it exceeds, stop the run, execute the next stage of
             * Twiddling and reset the simulator.
             */
            if( ( cte > cte_thresh) || ( cte < -cte_thresh ) )
            {
              cout<<"Params chosen didnot work out, reset sim \n";

              /* Modify the Hyper params for the next iteration */
              tw.HypTuning(step_counter,false);

              /* Set the hyperparams of PID to the modified values */
              pid.SetHypParams(tw);

              pid.PrintHypParams();
              tw.PrintDPParams();

              /* Reset the count, the current error and Reset the simulator now */
              step_counter = 0;
              is_reset_pending = true;
              std::string msg = "42[\"reset\",{}]";
              ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
            }

            if( ( pid.isTune_Needed == true )  && (pid.algo == Twiddling ) )
            {
              if( ( step_counter == TOT_STEPS ) ){
                /* Set the best error before the hyperparam Tuning kicks off for the very first time*/
                if( !pid.isfirstTuned )
                {
                  //This setting acts as a baseline to start Twiddling hyp param tuning.
                  tw.setBestError();
                  pid.isfirstTuned = true;
                }

                /* Modify the Hyper params for the next iteration */
                int res = tw.HypTuning(step_counter,true);

                /* Set the hyperparams of PID to the modified values */
                pid.SetHypParams(tw);
                pid.PrintHypParams();
                tw.PrintDPParams();

                /* Print the best set of params as well */
                tw.PrintBestParams();

                /* This occurs when sum of dp params is less than threshol in which case
                 * stop tuning
                 */
                if( res == 0 )
                {
                  cout<<"Tuning Done \n";
                  cout<<"Params are:";
                  pid.PrintHypParams();
                  tw.PrintDPParams();
                  exit(0);
                }

                /* Reset the count, the current error and Reset the simulator now */
                cout<<"One set of params iterated, resetting SIM \n";
                step_counter = 0;
                is_reset_pending = true;
                std::string msg = "42[\"reset\",{}]";
                ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
              }
            }
          }

          steer_value = pid.TotalError();
          if( steer_value > 1 ) steer_value = 1.0;
          else if( steer_value < -1 ) steer_value = -1.0;

          // DEBUG
          //std::cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;

          json msgJson;
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = 0.3;  //formerly 0.3
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1)
    {
      res->end(s.data(), s.length());
    }
    else
    {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h,&is_reset_pending](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
    is_reset_pending = false;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port))
  {
    std::cout << "Listening to port " << port << std::endl;
  }
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
