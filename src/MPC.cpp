#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen-3.3/Eigen/Core"

using CppAD::AD;

//Predict about a second into the future
int N = 10;
double dt = 0.1;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

//The number of actuators we control - steering and throttle
const int NACTUATORS = 2;

//The number of waypoints returned by simulator for ideal path
const int NWAYPOINTS = 6;

const double ref_cte = 0.0;
const double ref_epsi = 0.0;
//NOTE: Using a low ref_v to account for slow
//frame-rate simulation environment I have.
const double ref_v = 30.0;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

class FG_eval {
 public:
  // Fitted polynomial coefficients
  Eigen::VectorXd coeffs;
  FG_eval(Eigen::VectorXd coeffs) { this->coeffs = coeffs; }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    // MPC Implementation
    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)

    double kCte = 2000.0, kEpsi = 2000.0, kV = 1.0;
    double kDelta = 5.0, kA = 5.0;
    double kDeltaDiff = 200.0, kADiff = 10.0;

    // --- PART I - Cost Setup ---

    fg[0] = 0;

    // Reference State Cost
    // Define the cost related the reference state and
    // any anything you think may be beneficial.
    AD<double> cost = 0;
    //minimize errors
    for (int i = 0; i < N; i++) {
      cost += (kCte * CppAD::pow(vars[cte_start + i] - ref_cte, 2));
      cost += (kEpsi * CppAD::pow(vars[epsi_start + i] - ref_epsi, 2));
      cost += (kV * CppAD::pow(vars[v_start + i] - ref_v, 2));
    }
    //minimize control inputs (actuators)
    for (int i = 0; i < N - 1; i++) {
      cost += (kDelta * CppAD::pow(vars[delta_start + i], 2));
      cost += (kA * CppAD::pow(vars[a_start + i], 2));
    }
    //minimize sudden changes
    for (int i = 0; i < N - 2; i++) {
      cost += (kDeltaDiff * (CppAD::pow(vars[delta_start + i] - vars[delta_start + i + 1], 2)));
      cost += (kADiff * (CppAD::pow(vars[a_start + i] - vars[a_start + i + 1], 2)));
    }
    fg[0] = cost;

    // PART II - Constraints Setup

    /*
     * Initial constraints: We add 1 to each of the starting indices due to cost
     * being located at index 0 of `fg`. This bumps up the position of all the
     * other values.
     */
    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];

    // The rest of the constraints
    for (int t = 1; t < N; t++) {
      AD<double> x1 = vars[x_start + t];
      AD<double> x0 = vars[x_start + t - 1];
      AD<double> y1 = vars[x_start + t];
      AD<double> y0 = vars[x_start + t - 1];
      AD<double> psi1 = vars[psi_start + t];
      AD<double> psi0 = vars[psi_start + t - 1];
      AD<double> v1 = vars[v_start + t];
      AD<double> v0 = vars[v_start + t - 1];
      AD<double> cte1 = vars[cte_start + t];
      AD<double> epsi1 = vars[epsi_start + t];
      AD<double> epsi0 = vars[epsi_start + t - 1];
      AD<double> delta0 = vars[delta_start + t -1];
      AD<double> a0 = vars[a_start + t -1];

      // The idea here is to constraint this value to be 0.
      fg[1 + x_start + t] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[1 + y_start + t] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[1 + psi_start + t] = psi1 - (psi0 + ((v0 / Lf) * delta0 * dt));
      fg[1 + v_start + t] = v1 - (v0 + a0 * dt);
      AD<double> cte_des = coeffs[0] + coeffs[1] * x0 + coeffs[2] * x0 * x0 + coeffs[3] * x0 * x0 * x0;
      fg[1 + cte_start + t] = cte1 - ((cte_des - y0) + v0 * CppAD::sin(epsi0) * dt);
      AD<double> epsi_des = CppAD::atan(coeffs[1] + 2 * x0 * coeffs[2] + 3 * x0 * x0 * coeffs[3]);
      fg[1 + epsi_start + t] = epsi1 - ((psi0 - epsi_des) + ((v0 / Lf) * delta0 * dt));
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {}
MPC::~MPC() {}

vector<double> MPC::Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs) {
  bool ok = true;
  typedef CPPAD_TESTVECTOR(double) Dvector;

  // Set the number of model variables (includes both states and inputs).
  // For example: If the state is a 4 element vector, the actuators is a 2
  // element vector and there are 10 timesteps. The number of variables is:
  //
  // 4 * 10 + 2 * 9
  // Note that for N timesteps, there will only be N-1 actuations.
  int n_vars = state.size() * N + (N-1) * NACTUATORS;
  // Set the number of constraints
  int n_constraints = NWAYPOINTS * N;

  double x = state[0];
  double y = state[1];
  double psi = state[2];
  double v = state[3];
  double cte = state[4];
  double epsi = state[5];

  // Initial value of the independent variables.
  // SHOULD BE 0 besides initial state.
  Dvector vars(n_vars);
  for (int i = 0; i < n_vars; i++) {
    vars[i] = 0.0;
  }
  vars[x_start] = x;
  vars[y_start] = y;
  vars[psi_start] = psi;
  vars[v_start] = v;
  vars[cte_start] = cte;
  vars[epsi_start] = epsi;

  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of delta are set to -25 and 25
  // degrees (values in radians).
  // NOTE: Feel free to change this to something else.
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -0.436332;
    vars_upperbound[i] = 0.436332;
  }

  // Acceleration/decceleration upper and lower limits.
  // NOTE: Feel free to change this to something else.
  for (int i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = -1.0;
    vars_upperbound[i] = 1.0;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (int i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }
  constraints_lowerbound[x_start] = x;
  constraints_lowerbound[y_start] = y;
  constraints_lowerbound[psi_start] = psi;
  constraints_lowerbound[v_start] = v;
  constraints_lowerbound[cte_start] = cte;
  constraints_lowerbound[epsi_start] = epsi;

  constraints_upperbound[x_start] = x;
  constraints_upperbound[y_start] = y;
  constraints_upperbound[psi_start] = psi;
  constraints_upperbound[v_start] = v;
  constraints_upperbound[cte_start] = cte;
  constraints_upperbound[epsi_start] = epsi;

  // object that computes objective and constraints
  FG_eval fg_eval(coeffs);

  //
  // NOTE: You don't have to worry about these options
  //
  // options for IPOPT solver
  std::string options;
  // Uncomment this if you'd like more print information
  options += "Integer print_level  0\n";
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Change this as you see fit.
  options += "Numeric max_cpu_time          0.5\n";

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  ok &= solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  // Cost
  auto cost = solution.obj_value;
  std::cout << "Cost " << cost << std::endl;

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`. I'm returning the steering and throttle outputs along
  // with computed waypoints x and y coordinate values.
  vector<double> results;
  results.push_back(solution.x[delta_start]);
  results.push_back(solution.x[a_start]);
  for (int i = 1; i < N-1; i++) {
    results.push_back(solution.x[x_start + i]);
    results.push_back(solution.x[y_start + i]);
  }
  return results;
}
