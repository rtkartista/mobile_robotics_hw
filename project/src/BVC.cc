// get a target
// use ifpopt
// generate a setpoint for the controller
// publish the setpoint 
# include <iostream>
# include <ifopt/problem.h>
# include <ifopt/ipopt_solver.h>
// #include <ifopt/test_vars_constr_cost.h>
#include <problem_formulation.h>
using namespace ifopt;
int main() {

  // Define the solver independent problem
  Problem nlp;
  nlp.AddVariableSet  (std::make_shared<ExVariables>(10));
  nlp.AddConstraintSet(std::make_shared<ExConstraint>());
  nlp.AddCostSet      (std::make_shared<ExCost>());

  // Initialize solver and options
  IpoptSolver ipopt;
  ipopt.SetOption("linear_solver", "mumps");
  ipopt.SetOption("jacobian_approximation", "finite difference-values");
  ipopt.SetOption("max_iter", 6000);
  ipopt.SetOption("tol", 0.1);

  // Solve
  ipopt.Solve(nlp);

  std::cout << nlp.GetOptVariables()->GetValues().transpose() << std::endl;
}
