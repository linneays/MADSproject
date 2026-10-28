/*
  _____ _ _ _                    _             _
 |  ___(_) | |_ ___ _ __   _ __ | |_   _  __ _(_)_ __
 | |_  | | | __/ _ \ '__| | '_ \| | | | |/ _` | | '_ \
 |  _| | | | ||  __/ |    | |_) | | |_| | (_| | | | | |
 |_|   |_|_|\__\___|_|    | .__/|_|\__,_|\__, |_|_| |_|
                          |_|            |___/
# Ego-state complementary filter for the robotic walker
# Robotics Perception and Action - Courses Project, Complementary Filtering
# Based on the MADS 1.4.0 filter plugin template (my_filter.cpp)
#
# Inputs (sub_topic):  driver, calibrated_imu, htc
# Output (pub_topic):  ego_state_<group number>   (set in the .ini file)
#
# The perception loop follows the flowchart on slide 41:
#   1. Encoder increments             -> ds_E, ddelta_E
#   2. Gyro increment                 -> ddelta_G
#   3. Maneuver-dependent weights     -> mu, rho, xi_R
#   4. Encoder-gyro attitude fusion   -> ddelta_EG
#   5. Relative attitude estimate     -> delta_EG
#   6. Attitude complementary filter  -> delta_F
#   7. Encoder position estimate      -> x_E, y_E
#   8. Position complementary filter (x) -> x_F
#   9. Position complementary filter (y) -> y_F
#  10. Fused pose                     -> X_F = [x_F, y_F, delta_F]
#  11. Update stored states for the next iteration
*/
// Mandatory included headers
#include <filter.hpp>
#include <nlohmann/json.hpp>
#include <pugg/Kernel.h>

// other includes as needed here
#include <array>
#include <cmath>
#include <algorithm>

// Define the name of the plugin
#ifndef PLUGIN_NAME
#define PLUGIN_NAME "ego_state_filter"
#endif

// Load the namespaces
using namespace std;
using json = nlohmann::json;


/*
  _   _      _
 | | | | ___| |_ __   ___ _ __ ___
 | |_| |/ _ \ | '_ \ / _ \ '__/ __|
 |  _  |  __/ | |_) |  __/ |  \__ \
 |_| |_|\___|_| .__/ \___|_|  |___/
              |_|
*/

// Wrap an angle to (-pi, pi]
static double wrap(double angle) {
  return atan2(sin(angle), cos(angle));
}

// Third-order discrete IIR filter (slide 32):
//   y_k = -sum_{i=1..3} a_i y_{k-i} + sum_{i=0..3} b_i x_{k-i}
// Coefficients come from MATLAB c2d(..., 'tustin') + tfdata(..., 'v').
struct Iir3 {
  array<double, 4> b = {1.0, 0.0, 0.0, 0.0};
  array<double, 4> a = {1.0, 0.0, 0.0, 0.0};
  array<double, 3> x_hist = {0.0, 0.0, 0.0}; // x_{k-1}, x_{k-2}, x_{k-3}
  array<double, 3> y_hist = {0.0, 0.0, 0.0}; // y_{k-1}, y_{k-2}, y_{k-3}

  void set_coefficients(array<double, 4> const &b_new, array<double, 4> const &a_new) {
    // normalize so that a_0 = 1
    for (int i = 0; i < 4; i++) {
      b[i] = b_new[i] / a_new[0];
      a[i] = a_new[i] / a_new[0];
    }
  }

  // Fill the histories with a steady state, to avoid a start-up transient
  void reset(double input, double output) {
    x_hist.fill(input);
    y_hist.fill(output);
  }

  double step(double input) {
    double output = b[0] * input;
    for (int i = 1; i <= 3; i++) {
      output += b[i] * x_hist[i - 1] - a[i] * y_hist[i - 1];
    }
    x_hist = {input, x_hist[0], x_hist[1]};
    y_hist = {output, y_hist[0], y_hist[1]};
    return output;
  }
};

// Complementary filter: F = H_HP{relative estimate} + H_LP{absolute measure}
// with H_HP + H_LP = 1 (i.e. b_hp + b_lp = a)
struct ComplementaryFilter {
  Iir3 hp; // high-pass branch: encoder / encoder-gyro estimate
  Iir3 lp; // low-pass branch: HTC absolute measurement

  void set_coefficients(array<double, 4> const &b_hp,
                        array<double, 4> const &b_lp,
                        array<double, 4> const &a) {
    hp.set_coefficients(b_hp, a);
    lp.set_coefficients(b_lp, a);
  }

  // Steady state with both inputs equal to value: HP output 0, LP output value
  void reset(double value) {
    hp.reset(value, 0.0);
    lp.reset(value, value);
  }

  double step(double hp_input, double lp_input) {
    return hp.step(hp_input) + lp.step(lp_input);
  }
};

// Read a 4-element coefficient array from a json object, with a default
static array<double, 4> read_coeffs(json const &j, string const &key,
                                    array<double, 4> const &def) {
  if (!j.contains(key) || !j[key].is_array() || j[key].size() != 4) return def;
  array<double, 4> c;
  for (int i = 0; i < 4; i++) c[i] = j[key][i].get<double>();
  return c;
}


// Plugin class. This shall be the only part that needs to be modified,
// implementing the actual functionality
class EgoStateFilterPlugin : public Filter<json, json> {

public:

  // Typically, no need to change this
  string kind() override { return PLUGIN_NAME; }

  // Store the latest message of each input topic.
  // The perception loop is triggered by the driver (encoder) topic.
  return_type load_data(json const &input, string topic = "") override {
    if (topic == "driver") {
      // TODO: check if the left encoder has an inverted sign (mirrored motor)
      if (!input.contains("encoders")) return return_type::error;
      _n_L = input["encoders"].value("left", 0.0);
      _n_R = input["encoders"].value("right", 0.0);
      _driver_time = input.value("timecode", 0.0);
      _new_driver = true;
    }
    else if (topic == "calibrated_imu") {
      _omega_G = input.value("gyro", 0.0);
      _imu_received = true;
    }
    else if (topic == "htc") {
      // Htracker = midpoint between the wheels (Htracker_raw = tracker itself)
      // TODO: verify which position components are the planar x, y and
      //       which Euler angle is the yaw, in the HTC reference frame
      if (!input.contains("Htracker")) return return_type::error;
      json const &pose = input["Htracker"]["pose"];
      _x_H = pose["position"][0].get<double>();
      _y_H = pose["position"][1].get<double>();
      _delta_H_raw = pose["attitude"][2].get<double>();
      _htc_received = true;
    }
    return return_type::success;
  }

  // One iteration k-1 -> k of the perception loop (slide 41)
  return_type process(json &out) override {
    out.clear();

    // Run only once per new encoder sample
    // TODO: check that return_type::retry makes the agent skip publishing
    if (!_new_driver) return return_type::retry;
    _new_driver = false;

    // Wait for an absolute HTC measurement to initialize the states
    if (!_initialized) {
      if (!_htc_received) return return_type::retry;
      initialize();
      return return_type::retry;
    }

    // Perception-loop period (measured from the driver timecode)
    double Tc = _driver_time - _driver_time_prev;
    if (Tc <= 0.0) Tc = _Tc; // fallback to the nominal period

    // HTC attitude made continuous, so it can be filtered together with delta_EG
    double delta_H = unwrap_htc_attitude(_delta_H_raw);

    // ---- 1. Encoder increments ------------------------------------------
    double dn_R = _n_R - _n_R_prev;
    double dn_L = _n_L - _n_L_prev;
    double ds_R = 2.0 * M_PI * _wheel_radius / _counts_per_rev * dn_R;
    double ds_L = 2.0 * M_PI * _wheel_radius / _counts_per_rev * dn_L;
    double ds_E = (ds_R + ds_L) / 2.0;
    double ddelta_E = (ds_R - ds_L) / _wheel_base;

    // ---- 2. Gyro increment ----------------------------------------------
    // TODO: estimate b_G (e.g. while standing, or with another MADS agent)
    double ddelta_G = Tc * (_omega_G - _gyro_bias);

    // ---- 3. Maneuver-dependent weights ----------------------------------
    double mu = turning_index(ds_R, ds_L);
    double rho = motion_index(ds_R, ds_L);
    double xi_R = accuracy_ratio(mu, rho);

    // ---- 4. Encoder-gyro attitude fusion --------------------------------
    double w_E = 1.0 / (1.0 + xi_R * xi_R);
    double w_G = xi_R * xi_R / (1.0 + xi_R * xi_R);
    double ddelta_EG = w_E * ddelta_E + w_G * ddelta_G;

    // ---- 5. Relative attitude estimate ----------------------------------
    double delta_EG = _delta_EG_prev + ddelta_EG;

    // ---- 6. Attitude complementary filter -------------------------------
    double delta_F = _attitude_filter.step(delta_EG, delta_H);

    // ---- 7. Encoder position estimate -----------------------------------
    double delta_F_mean = (_delta_F_prev + delta_F) / 2.0;
    double x_E = _x_E_prev + ds_E * cos(delta_F_mean);
    double y_E = _y_E_prev + ds_E * sin(delta_F_mean);

    // ---- 8. Position complementary filter (x) ---------------------------
    double x_F = _x_filter.step(x_E, _x_H);

    // ---- 9. Position complementary filter (y) ---------------------------
    double y_F = _y_filter.step(y_E, _y_H);

    // ---- 10. Fused pose -------------------------------------------------
    write_output(out, x_F, y_F, wrap(delta_F));

    // Extra fields, useful for debugging/plotting the single branches
    out["debug"] = {
      {"ds_E", ds_E}, {"ddelta_E", ddelta_E}, {"ddelta_G", ddelta_G},
      {"mu", mu}, {"rho", rho}, {"xi_R", xi_R},
      {"delta_EG", delta_EG}, {"delta_H", delta_H},
      {"x_E", x_E}, {"y_E", y_E}, {"x_H", _x_H}, {"y_H", _y_H}
    };

    // ---- 11. Update stored states for next iteration --------------------
    // (filter histories are updated inside Iir3::step)
    _n_R_prev = _n_R;
    _n_L_prev = _n_L;
    _driver_time_prev = _driver_time;
    _delta_EG_prev = delta_EG;
    _delta_F_prev = delta_F;
    _x_E_prev = x_E;
    _y_E_prev = y_E;

    if (!_agent_id.empty()) out["agent_id"] = _agent_id;
    return return_type::success;
  }

  void set_params(void const *params) override {
    // Call the parent class method to set the common parameters
    // (e.g. agent_id, etc.)
    Filter::set_params(params);

    // Defaults first, then merge the actually provided parameters,
    // then read the values (so the .ini file overrides the defaults)
    // TODO: replace the dummy values with the real walker parameters
    _params["wheel_radius"] = 0.1;     // [m]
    _params["wheel_base"] = 0.5;       // [m]
    _params["counts_per_rev"] = 4096;  // [tics/rev]
    _params["imu"]["sigma_gyro"] = 0.01; // [rad/s]
    _params["gyro_bias"] = 0.0;        // [rad/s]
    _params["Tc"] = 0.03;              // [s] nominal perception-loop period

    // Dummy filter coefficients: HP branch = 0, LP branch = 1, i.e. the
    // output is just the HTC measurement. Replace with the MATLAB values
    // (bh_*, bl_*, ah_*). Complementarity requires b_hp + b_lp = a.
    _params["attitude_filter"] = {
      {"a",    {1.0, 0.0, 0.0, 0.0}},
      {"b_hp", {0.0, 0.0, 0.0, 0.0}},
      {"b_lp", {1.0, 0.0, 0.0, 0.0}}
    };
    _params["position_filter"] = _params["attitude_filter"];

    // params needs to be cast to json
    _params.merge_patch(*(json *)params);

    _wheel_radius = _params["wheel_radius"].get<double>();
    _wheel_base = _params["wheel_base"].get<double>();
    _counts_per_rev = _params["counts_per_rev"].get<double>();
    _sigma_gyro = _params["imu"]["sigma_gyro"].get<double>();
    _gyro_bias = _params["gyro_bias"].get<double>();
    _Tc = _params["Tc"].get<double>();

    array<double, 4> one = {1.0, 0.0, 0.0, 0.0};
    array<double, 4> zero = {0.0, 0.0, 0.0, 0.0};

    json const &att = _params["attitude_filter"];
    _attitude_filter.set_coefficients(read_coeffs(att, "b_hp", zero),
                                      read_coeffs(att, "b_lp", one),
                                      read_coeffs(att, "a", one));

    json const &pos = _params["position_filter"];
    _x_filter.set_coefficients(read_coeffs(pos, "b_hp", zero),
                               read_coeffs(pos, "b_lp", one),
                               read_coeffs(pos, "a", one));
    _y_filter.set_coefficients(read_coeffs(pos, "b_hp", zero),
                               read_coeffs(pos, "b_lp", one),
                               read_coeffs(pos, "a", one));
  }

  // Implement this method if you want to provide additional information
  map<string, string> info() override {
    return {
      {"wheel_radius", to_string(_wheel_radius)},
      {"wheel_base", to_string(_wheel_base)},
      {"sigma_gyro", to_string(_sigma_gyro)},
      {"Tc", to_string(_Tc)}
    };
  };

private:

  // ---- Initialization ---------------------------------------------------
  // Start all relative estimates from the first HTC pose, and fill the
  // filter histories with a steady state
  void initialize() {
    _n_R_prev = _n_R;
    _n_L_prev = _n_L;
    _driver_time_prev = _driver_time;

    _delta_H_prev = _delta_H_raw;
    _delta_H_unwrapped = _delta_H_raw;

    _delta_EG_prev = _delta_H_raw;
    _delta_F_prev = _delta_H_raw;
    _x_E_prev = _x_H;
    _y_E_prev = _y_H;

    _attitude_filter.reset(_delta_H_raw);
    _x_filter.reset(_x_H);
    _y_filter.reset(_y_H);

    _initialized = true;
  }

  // Remove the +-pi jumps from the HTC yaw angle
  double unwrap_htc_attitude(double delta_H_raw) {
    _delta_H_unwrapped += wrap(delta_H_raw - _delta_H_prev);
    _delta_H_prev = delta_H_raw;
    return _delta_H_unwrapped;
  }

  // ---- Step 3 helpers ---------------------------------------------------
  // mu_k: 0 = straight path, 1 = turning on the spot (slide 22)
  double turning_index(double ds_R, double ds_L) {
    return fabs(ds_R - ds_L) / (fabs(ds_R) + fabs(ds_L) + _epsilon);
  }

  // rho_k = f_rho(|ds_R| + |ds_L|), in [0, 1]: 0 = standing, 1 = moving
  double motion_index(double ds_R, double ds_L) {
    double s = fabs(ds_R) + fabs(ds_L);
    // TODO: define the continuous function f_rho
    (void)s;
    return 1.0;
  }

  // xi_R = f_xi(mu, rho) = sigma_E / sigma_G (slides 22, 25-27)
  // Must be a smooth function (no discrete steps as in the 2003 paper)
  double accuracy_ratio(double mu, double rho) {
    // TODO: define the continuous function f_xi, e.g. interpolating the
    //       values of Table 1 (standing 0, straight 0.25, curving 2,
    //       steering 4). _sigma_gyro can be used for sigma_G.
    (void)mu;
    (void)rho;
    return 1.0; // dummy: equal weights for encoder and gyro
  }

  // ---- Step 10 helper ---------------------------------------------------
  // Fused pose as a 4x4 homogeneous transformation (slide 52)
  // TODO: compare the rotation convention with the reference ego_state topic
  void write_output(json &out, double x, double y, double delta) {
    double c = cos(delta), s = sin(delta);
    out["H"] = {
      {c,  -s,  0.0, x  },
      {s,   c,  0.0, y  },
      {0.0, 0.0, 1.0, 0.0},
      {0.0, 0.0, 0.0, 1.0}
    };
    out["pose"] = {{"x", x}, {"y", y}, {"delta", delta}};
  }

  // ---- Parameters -------------------------------------------------------
  double _wheel_radius = 0.1;
  double _wheel_base = 0.5;
  double _counts_per_rev = 4096;
  double _sigma_gyro = 0.01;
  double _gyro_bias = 0.0;
  double _Tc = 0.03;
  double _epsilon = 1e-9;

  // ---- Latest input data ------------------------------------------------
  double _n_R = 0.0, _n_L = 0.0;   // cumulative encoder counts
  double _driver_time = 0.0;       // driver timecode [s]
  double _omega_G = 0.0;           // gyro yaw rate [rad/s]
  double _x_H = 0.0, _y_H = 0.0;   // HTC position [m]
  double _delta_H_raw = 0.0;       // HTC yaw [rad], wrapped
  bool _new_driver = false;
  bool _imu_received = false;
  bool _htc_received = false;
  bool _initialized = false;

  // ---- States at k-1 ----------------------------------------------------
  double _n_R_prev = 0.0, _n_L_prev = 0.0;
  double _driver_time_prev = 0.0;
  double _delta_EG_prev = 0.0;
  double _delta_F_prev = 0.0;
  double _x_E_prev = 0.0, _y_E_prev = 0.0;
  double _delta_H_prev = 0.0, _delta_H_unwrapped = 0.0;

  // ---- Complementary filters --------------------------------------------
  ComplementaryFilter _attitude_filter;
  ComplementaryFilter _x_filter;
  ComplementaryFilter _y_filter;
};


/*
  ____  _             _             _      _
 |  _ \| |_   _  __ _(_)_ __     __| |_ __(_)_   _____ _ __
 | |_) | | | | |/ _` | | '_ \   / _` | '__| \ \ / / _ \ '__|
 |  __/| | |_| | (_| | | | | | | (_| | |  | |\ V /  __/ |
 |_|   |_|\__,_|\__, |_|_| |_|  \__,_|_|  |_| \_/ \___|_|
                |___/
Enable the class as plugin
*/
INSTALL_FILTER_DRIVER(EgoStateFilterPlugin, json, json);


/*
                  _
  _ __ ___   __ _(_)_ __
 | '_ ` _ \ / _` | | '_ \
 | | | | | | (_| | | | | |
 |_| |_| |_|\__,_|_|_| |_|

Quick standalone test with fake messages
*/

int main(int argc, char const *argv[])
{
  EgoStateFilterPlugin plugin;
  json params, output;

  params["wheel_radius"] = 0.1;
  params["wheel_base"] = 0.5;
  plugin.set_params(&params);

  json htc;
  htc["Htracker"]["pose"]["position"] = {0.0, 0.0, 0.0};
  htc["Htracker"]["pose"]["attitude"] = {0.0, 0.0, 0.0};

  json imu;
  imu["gyro"] = 0.0;

  // Drive straight: both encoders advance by 100 tics per sample
  for (int k = 0; k < 5; k++) {
    json driver;
    driver["encoders"] = {{"left", 100 * k}, {"right", 100 * k}};
    driver["timecode"] = 0.03 * k;

    plugin.load_data(htc, "htc");
    plugin.load_data(imu, "calibrated_imu");
    plugin.load_data(driver, "driver");
    if (plugin.process(output) == return_type::success)
      cout << "k = " << k << ": " << output["pose"].dump() << endl;
  }

  return 0;
}
