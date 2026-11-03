/*
  _____ _ _ _                    _             _
 |  ___(_) | |_ ___ _ __   _ __ | |_   _  __ _(_)_ __
 | |_  | | | __/ _ \ '__| | '_ \| | | | |/ _` | | '_ \
 |  _| | | | ||  __/ |    | |_) | | |_| | (_| | | | | |
 |_|   |_|_|\__\___|_|    | .__/|_|\__,_|\__, |_|_| |_|
                          |_|            |___/
# Gyroscope bias estimator for the robotic walker
# Robotics Perception and Action - Courses Project, Complementary Filtering
# Based on the MADS 1.4.0 filter plugin template (my_filter.cpp)
#
# Inputs (sub_topic):  driver, calibrated_imu
# Output (pub_topic):  gyro_bias   (set in the .ini file)
#
#
#   1. Standstill detection: the encoder counts have not changed for
#      still_min consecutive driver messages
#   2. While standing still, the gyro should read 0: every reading is
#      bias + noise
#   3. Running average of these readings (exponential moving average, so
#      the estimate follows slow drift, e.g. with temperature)
#   4. Publish the estimate; nothing is published while the walker moves
*/
// Mandatory included headers
#include <filter.hpp>
#include <nlohmann/json.hpp>
#include <pugg/Kernel.h>

// other includes as needed here
#include <cmath>
#include <algorithm>

// Define the name of the plugin
#ifndef PLUGIN_NAME
#define PLUGIN_NAME "gyro_bias_agent"
#endif

// Load the namespaces
using namespace std;
using json = nlohmann::json;


// Plugin class. This shall be the only part that needs to be modified,
// implementing the actual functionality
class GyroBiasAgentPlugin : public Filter<json, json> {

public:

  // Typically, no need to change this
  string kind() override { return PLUGIN_NAME; }

  // driver: standstill detection. calibrated_imu: store the gyro reading,
  // which triggers one update in process()
  return_type load_data(json const &input, string topic = "", vector<unsigned char> const *blob = nullptr) override {
    if (topic == "driver") {
      if (!input.contains("encoders")) return return_type::error;
      double n_L = input["encoders"].value("left", 0.0);
      double n_R = input["encoders"].value("right", 0.0);

      // ---- 1. Standstill detection --------------------------------------
      // The first message has nothing to compare with: treat it as moving
      if (_driver_received && n_L == _n_L_prev && n_R == _n_R_prev)
        _still_count++;
      else
        _still_count = 0;

      _n_L_prev = n_L;
      _n_R_prev = n_R;
      _driver_received = true;
    }
    else if (topic == "calibrated_imu") {
      _omega_G = input.value("gyro", 0.0);
      _new_imu = true;
    }
    return return_type::success;
  }

  // One update of the bias estimate per new gyro sample
  return_type process	(json &out, vector<unsigned char> *blob = nullptr) override {
    out.clear();

    // Run only once per new gyro sample
    if (!_new_imu) return return_type::retry;
    _new_imu = false;

    // Only estimate after standing still long enough 
    if (_still_count < _still_min) return return_type::retry;

    // ---- 2. Reject readings that cannot be bias + noise ------------------
    // e.g. someone turns the walker without the wheels rolling
    double delta = _omega_G - _bias;
    if (_n > 0 && fabs(delta) > _max_rate) return return_type::retry;

    // ---- 3. Running average ----------------------------------------------
    // alpha = 1/n at the start (plain mean, converges fast), then Ts/tau
    // (exponential moving average with a memory of about tau seconds)
    _n++;
    double alpha = max(1.0 / _n, _imu_period / _tau);
    _bias += alpha * delta;
    _variance = (1.0 - alpha) * (_variance + alpha * delta * delta);

    // ---- 4. Publish ------------------------------------------------------
    out["bias"] = _bias;                 // [rad/s] -> b_G in the ego filter
    out["sigma"] = sqrt(_variance);      // [rad/s] gyro noise while standing
    out["n_samples"] = _n;

    if (!_agent_id.empty()) out["agent_id"] = _agent_id;
    return return_type::success;
  }

  void set_params(const json &params) override {
    // Call the parent class method to set the common parameters
    // (e.g. agent_id, etc.)
    Filter::set_params(params);

    // Defaults first, then merge the actually provided parameters,
    // then read the values (so the .ini file overrides the defaults)
    _params["still_min"] = _still_min;       // [driver messages] ~0.5 s at 8 ms
    _params["imu_period"] = _imu_period;     // [s] calibrated_imu period
    _params["tau"] = _tau;                   // [s] memory of the average
    _params["max_rate"] = _max_rate;         // [rad/s] outlier threshold

    // params needs to be cast to json
    _params.merge_patch(params);

    _still_min = _params["still_min"].get<int>();
    _imu_period = _params["imu_period"].get<double>();
    _tau = _params["tau"].get<double>();
    _max_rate = _params["max_rate"].get<double>();
  }

  // Implement this method if you want to provide additional information
  map<string, string> info() override {
    return {
      {"still_min", to_string(_still_min)},
      {"imu_period", to_string(_imu_period)},
      {"tau", to_string(_tau)},
      {"max_rate", to_string(_max_rate)}
    };
  };

private:

  // ---- Parameters -------------------------------------------------------
  int _still_min = 60;
  double _imu_period = 0.02;
  double _tau = 10.0;
  double _max_rate = 0.05;

  // ---- Standstill detection ---------------------------------------------
  double _n_L_prev = 0.0, _n_R_prev = 0.0;
  bool _driver_received = false;
  int _still_count = 0;

  // ---- Latest gyro reading ----------------------------------------------
  double _omega_G = 0.0;
  bool _new_imu = false;

  // ---- Estimate ---------------------------------------------------------
  double _bias = 0.0;
  double _variance = 0.0;
  long _n = 0;
};


/*
  ____  _             _             _      _
 |  _ \| |_   _  __ _(_)_ __     __| |_ __(_)_   _____ _ __
 | |_) | | | | |/ _` | | '_ \   / _` | '__| \ \ / / _ \ '__|
 |  __/| | |_| | (_| | | | | | | (_| | |  | |\ V /  __/ |
 |_|   |_|_|\__\___|_|    | .__/|_|\__,_|\__, |_|_| |_|
                |___/
Enable the class as plugin
*/
MADS_REGISTER_PLUGINS(GyroBiasAgentPlugin);


/*
                  _
  _ __ ___   __ _(_)_ __
 | '_ ` _ \ / _` | | '_ \
 | | | | | | (_| | | | | |
 |_| |_| |_|\__,_|_|_| |_|

Quick standalone test with fake messages: the walker stands still and the
gyro reads a bias of -0.0005 rad/s plus alternating noise
*/

int main(int argc, char const *argv[])
{
  GyroBiasAgentPlugin plugin;
  json params, output;

  params["still_min"] = 10;
  plugin.set_params(params);

  json driver;
  driver["encoders"] = {{"left", 1863}, {"right", 3581}};

  for (int k = 0; k < 40; k++) {
    json imu;
    imu["gyro"] = -0.0005 + ((k % 2 == 0) ? 0.002 : -0.002);

    plugin.load_data(driver, "driver");
    plugin.load_data(imu, "calibrated_imu");
    if (plugin.process(output) == return_type::success)
      cout << "k = " << k << ": " << output.dump() << endl;
  }

  return 0;
}
