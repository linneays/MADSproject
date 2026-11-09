/*
Gyro bias agent
Inputs  (sub_topic): driver (encoders), calibrated_imu (gyro)
Output  (pub_topic): gyro_bias -> { gyro_bias }
Idea:
  - standing still:   the true rotation is 0, so the gyro reading is the bias
                        -update the estimate (running average)
  - moving: do not update, just keep sending the last estimate
*/
#include <filter.hpp>
#include <nlohmann/json.hpp>
#include <pugg/Kernel.h>
#include <cmath>

#ifndef PLUGIN_NAME
#define PLUGIN_NAME "gyro_bias_agent"
#endif

using namespace std;
using json = nlohmann::json;


class GyroBiasAgentPlugin : public Filter<json, json> {

public:

  string kind() override { return PLUGIN_NAME; }

  // Called for every incoming message: only store the values
  return_type load_data(json const &input, string topic = "", vector<unsigned char> const *blob = nullptr) override {

    if (topic == "driver") {
      double n_L = input["encoders"].value("left", 0.0);
      double n_R = input["encoders"].value("right", 0.0);

      // Count how many driver messages in a row have unchanged encoders
      if (_have_encoders && n_L == _n_L_prev && n_R == _n_R_prev)
        _still_count++;
      else
        _still_count = 0;   // any change in the encoders = we are moving

      _n_L_prev = n_L;
      _n_R_prev = n_R;
      _have_encoders = true;
    }
    else if (topic == "calibrated_imu") {
      _gyro = input.value("gyro", 0.0);
      _new_gyro = true;
    }
    return return_type::success;
  }

  // Called once per new gyro sample
  return_type process(json &out, vector<unsigned char> *blob = nullptr) override {
    out.clear();

    if (!_new_gyro) return return_type::retry;
    _new_gyro = false;

    // 1. Are we standing still? (encoders unchanged for long enough)
    bool standing = (_still_count >= _still_min);

    // 2. Only update the estimate while standing
    if (standing) {
      if (!_has_estimate) {
        _bias = _gyro;                    // first estimate: take the reading
        _has_estimate = true;
      } else {
        _bias += _alpha * (_gyro - _bias); // move a little towards the new reading
      }
    }

    // 3. Nothing to send until we have measured a bias at least once
    if (!_has_estimate) return return_type::retry;

    // 4. Always send the latest estimate (frozen while moving)
    out["gyro_bias"] = _bias;         // [rad/s]
   
    if (!_agent_id.empty()) out["agent_id"] = _agent_id;
    return return_type::success;
  }

  void set_params(const json &params) override {
    Filter::set_params(params);

    _params["still_min"] = _still_min;  // driver messages without encoder change
    _params["alpha"] = _alpha;          // averaging speed: small = slow and smooth
    _params.merge_patch(params);

    _still_min = _params["still_min"].get<int>();
    _alpha = _params["alpha"].get<double>();
  }

  map<string, string> info() override {
    return {};
  };

private:
  // parameters (can be overridden in the .ini file)
  int _still_min = 60;
  double _alpha = 0.01;    // each standing sample moves the estimate 1% of the way

  // encoder state
  double _n_L_prev = 0.0, _n_R_prev = 0.0;
  bool _have_encoders = false;
  int _still_count = 0;

  // gyro state
  double _gyro = 0.0;
  bool _new_gyro = false;

  // the estimate
  double _bias = 0.0;
  bool _has_estimate = false;
};


MADS_REGISTER_PLUGINS(GyroBiasAgentPlugin);


int main(int argc, char const *argv[])
{
  GyroBiasAgentPlugin plugin;
  json params, output;
  params["still_min"] = 10;
  plugin.set_params(params);

  int enc = 0;
  for (int k = 0; k < 300; k++) {
    bool moving = (k >= 100 && k < 200);
    if (moving) enc += 4;                       // encoders change = moving

    json driver, imu;
    driver["encoders"] = {{"left", enc}, {"right", enc}};
    imu["gyro"] = moving ? 0.3 : -0.004;        // large rate while moving

    plugin.load_data(driver, "driver");
    plugin.load_data(imu, "calibrated_imu");
    if (plugin.process(output) == return_type::success && k % 25 == 0)
      cout << "k = " << k << ": " << output.dump() << endl;
  }
  return 0;
}
