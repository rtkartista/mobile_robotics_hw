/*
 *  @file test_vars_constr_cost.h
*/

#include <ifopt/variable_set.h>
#include <ifopt/constraint_set.h>
#include <ifopt/cost_term.h>

namespace ifopt {
using Eigen::Vector3d;

class ExVariables : public VariableSet {
public:
  // Every variable set has a name, here "var_set1". this allows the constraints
  // and costs to define values and Jacobians specifically w.r.t this variable set.
  ExVariables() : ExVariables("var_set1") {};
  ExVariables(const std::string& name) : VariableSet(3, name)
  {

    // the initial values where the NLP starts iterating from
    // Current location (x, y, z)
    x0_ = 0;
    x1_ = 0;
    x2_ = 0;
  }
  // Here is where you can transform the Eigen::Vector into whatever
  // internal representation of your variables you have (here two doubles, but
  // can also be complex classes such as splines, etc..
  void SetVariables(const VectorXd& x) override
  {
    x0_ = x(0);
    x1_ = x(1);
    x2_ = x(2);
  };

  // Here is the reverse transformation from the internal representation to
  // to the Eigen::Vector
  VectorXd GetValues() const override
  {
    return Vector3d(x0_, x1_, x2_);
  };

  // Each variable has an upper and lower bound set here
  VecBound GetBounds() const override
  {
    VecBound bounds(GetRows());
    bounds.at(0) = Bounds(-8.0, 8.0);
    bounds.at(1) = Bounds(-8.0, 8.0);
    bounds.at(2) = Bounds(0.0, 0.1);
    return bounds;
  }

private:
  double x0_, x1_, x2_;
};


class ExConstraint : public ConstraintSet {
public:
    // Position of other UAV
  float x1c = 0.95;
  float y1c = 0.7;
  float z1c = 0;
  float xc = 0;
  float yc = 0;
  float zc = 0;
  float vxc = 0.5;
  float vyc = 0;
  float vzc = 0;
  // variance
  float sig = 0.2;
  //amax
  float amax = 0.1;
  // prob
  float del = 0.2;
  // safety radius
  float r = 0.1;

  ExConstraint() : ExConstraint("constraint1") {}

  // This constraint set just contains 1 constraint, however generally
  // each set can contain multiple related constraints.
  ExConstraint(const std::string& name) : ConstraintSet(1, name) {}
  void GetObsValues(){

  }
  // The constraint value minus the constant value "1", moved to bounds.
  VectorXd GetValues() const override
  {
    // hyperparameter calculations
    float a0 = (2/(sig*sig))*(x1c-xc);
    float a1 = (2/(sig*sig))*(y1c-yc);
    float a2 = (2/(sig*sig))*(z1c-zc);

    float b0 = (1/(sig*sig))*(((x1c-xc)*(x1c+xc))+((y1c-yc)*(y1c+yc))+((z1c-zc)*(z1c+zc)));
    float b1 = sqrt(a0*a0 + a1*a1 + a2*a2) * r;
    float b3 = (sqrt(3.14)*(2*sqrt(1-del)-1)/2) * sig * sqrt(2*(a0*a0 + a1*a1 + a2*a2));
    float b2 = 0;
    if(a0*vxc + a1*vyc + a2*vzc > 0){
        b2 = (a0*vxc + a1*vyc + a2*vzc)/(2*amax);
    }
    
    VectorXd g(GetRows());
    Vector3d x = GetVariables()->GetComponent("var_set1")->GetValues();
    // g(0) = std::pow(x(0),2) + x(1);
    g(0) = a0*x(0) + a1*x(1) + a2*x(2) - b0 + b1 + b2 + b3;

    return g;
  };

  // The only constraint in this set is an equality constraint to 1.
  // Constant values should always be put into GetBounds(), not GetValues().
  // For inequality constraints (<,>), use Bounds(x, inf) or Bounds(-inf, x).
  VecBound GetBounds() const override
  {
    VecBound b(GetRows());
    b.at(0) = Bounds(-0.1, 10000);
    return b;
  }

  // This function provides the first derivative of the constraints.
  // In case this is too difficult to write, you can also tell the solvers to
  // approximate the derivatives by finite differences and not overwrite this
  // function, e.g. in ipopt.cc::use_jacobian_approximation_ = true
  // Attention: see the parent class function for important information on sparsity pattern.
  void FillJacobianBlock (std::string var_set, Jacobian& jac_block) const override
  {
    // must fill only that submatrix of the overall Jacobian that relates
    // to this constraint and "var_set1". even if more constraints or variables
    // classes are added, this submatrix will always start at row 0 and column 0,
    // thereby being independent from the overall problem.

    if (var_set == "var_set1") {
      Vector3d x = GetVariables()->GetComponent("var_set1")->GetValues();

      jac_block.coeffRef(0, 0) = (2/(sig*sig))*(x1c-xc); // derivative of first constraint w.r.t x0
      jac_block.coeffRef(0, 1) = (2/(sig*sig))*(y1c-yc);      // derivative of first constraint w.r.t x1
      jac_block.coeffRef(0, 2) = (2/(sig*sig))*(z1c-zc);;
    }
  }
};


class ExCost: public CostTerm {
public:
  float xc = 0;
float yc = 0;
float zc = 0;
// goal
float xg = 5;
float yg = 2;
float zg = 0;
float Kdist0 = xc-xg;
float Kdist1 = yc-yg;
float Kdist2 = zc-zg;
float Kdist3= sqrt((xc-xg)*(xc-xg) + (yc-yg)*(yc-yg) + (zc-zg)*(zc-zg));
  
  ExCost() : ExCost("cost_term1") {}
  ExCost(const std::string& name) : CostTerm(name) {
  }

  double GetCost() const override
  {
    Vector3d x = GetVariables()->GetComponent("var_set1")->GetValues();
    return acos((x(0)*(Kdist0) + x(1)*(Kdist1) + x(2)*(Kdist2))/(sqrt(x(0)*x(0) + x(1)*x(1) + x(2)*x(2)) * Kdist3));
  };

  void FillJacobianBlock (std::string var_set, Jacobian& jac) const override
  {
    if (var_set == "var_set1") {
      Vector3d x = GetVariables()->GetComponent("var_set1")->GetValues();

    jac.coeffRef(0, 0) = ((((Kdist0/Kdist3)*(sqrt(x(0)*x(0) + x(1)*x(1) + x(2)*x(2)))) 
            + ((x(0)*((x(0)*Kdist0)+ (x(1)*Kdist1) + (x(2)*Kdist2)))/(Kdist3*sqrt(x(0)*x(0) + x(1)*x(1) + x(2)*x(2)))))
            /(1-((sqrt(x(0)*x(0) + x(1)*x(1) + x(2)*x(2))*
            (x(0)*Kdist0 + x(1)*Kdist1 + x(2)*Kdist2)*(x(0)*Kdist0 + x(1)*Kdist1 + x(2)*Kdist2))/(Kdist3*Kdist3))));

    jac.coeffRef(0, 1) = ((((Kdist1/Kdist3)*(sqrt(x(0)*x(0) + x(1)*x(1) + x(2)*x(2)))) 
                    + ((x(1)*((x(0)*Kdist0)+ (x(1)*Kdist1) + (x(2)*Kdist2)))/(Kdist3*sqrt(x(0)*x(0) + x(1)*x(1) + x(2)*x(2)))))
                    /(1-((sqrt(x(0)*x(0) + x(1)*x(1) + x(2)*x(2))*
                    (x(0)*Kdist0 + x(1)*Kdist1 + x(2)*Kdist2)*(x(0)*Kdist0 + x(1)*Kdist1 + x(2)*Kdist2))/(Kdist3*Kdist3))));
    
    jac.coeffRef(0, 2) = ((((Kdist2/Kdist3)*(sqrt(x(0)*x(0) + x(1)*x(1) + x(2)*x(2)))) 
                    + ((x(2)*((x(0)*Kdist0)+ (x(1)*Kdist1) + (x(2)*Kdist2)))/(Kdist3*sqrt(x(0)*x(0) + x(1)*x(1) + x(2)*x(2)))))
                    /(1-((sqrt(x(0)*x(0) + x(1)*x(1) + x(2)*x(2))*
                    (x(0)*Kdist0 + x(1)*Kdist1 + x(2)*Kdist2)*(x(0)*Kdist0 + x(1)*Kdist1 + x(2)*Kdist2))/(Kdist3*Kdist3))));
    
    }
  }
};

} // namespace opt
