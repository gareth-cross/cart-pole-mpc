// Copyright 2024 Gareth Cross.
// WASM module for interacting with the controller.
#include <emscripten/bind.h>

#include "integration.hpp"
#include "single_pendulum_dynamics.hpp"

#include "mini_opt/logging.hpp"
#include "mini_opt/nonlinear.hpp"
#include "mini_opt/residual.hpp"

namespace pendulum {

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

}  // namespace pendulum

namespace em = emscripten;

EMSCRIPTEN_BINDINGS(OptimizationWasm) { em::function("optimize", &pendulum::optimize); }
