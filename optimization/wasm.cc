// Copyright 2024 Gareth Cross.
// WASM module for interacting with the controller.
#include <emscripten/bind.h>

#include "simulator.hpp"

#include "mini_opt/logging.hpp"
#include "mini_opt/nonlinear.hpp"
#include "mini_opt/residual.hpp"

namespace pendulum {

#if 0
void optimize() {
  mini_opt::Problem problem{};
  problem.dimension = 1;
  mini_opt::ConstrainedNonlinearLeastSquares nls(&problem);
  mini_opt::ConstrainedNonlinearLeastSquares::Params p{};
  p.max_iterations = 100;
  p.relative_exit_tol = 0.0;
  p.equality_penalty_scale_factor = 1.01;
  p.max_qp_iterations = 1;
  p.max_line_search_iterations = 5;

  mini_opt::Logger logger{};
  nls.SetLoggingCallback(std::bind(&mini_opt::Logger::NonlinearSolverCallback, &logger,
                                   std::placeholders::_1, std::placeholders::_2));

  Eigen::VectorXd u_params = Eigen::VectorXd::Zero(1);
  const mini_opt::NLSSolverOutputs outputs = nls.Solve(p, u_params);
  fmt::print("number of iterations: {}\n", outputs.num_iterations);
}
#endif

}  // namespace pendulum

namespace em = emscripten;

EMSCRIPTEN_BINDINGS(OptimizationWasm) {
  em::class_<pendulum::SingleCartPoleState>("SingleCartPoleState")
      .constructor<double, double, double, double>()
      .property("b_x", &pendulum::SingleCartPoleState::b_x)
      .property("th_1", &pendulum::SingleCartPoleState::th_1)
      .property("b_x_dot", &pendulum::SingleCartPoleState::b_x_dot)
      .property("th_1_dot", &pendulum::SingleCartPoleState::th_1_dot);

  em::class_<pendulum::SingleCartPoleParams>("SingleCartPoleParams")
      .constructor<double, double, double, double>()
      .property("m_b", &pendulum::SingleCartPoleParams::m_b)
      .property("m_1", &pendulum::SingleCartPoleParams::m_1)
      .property("l_1", &pendulum::SingleCartPoleParams::l_1)
      .property("g", &pendulum::SingleCartPoleParams::g);

  em::class_<pendulum::Simulator>("Simulator")
      .constructor<pendulum::SingleCartPoleParams>()
      .function("step", &pendulum::Simulator::Step)
      .function("getState", &pendulum::Simulator::GetState)
      .function("setState", &pendulum::Simulator::SetState);
}
