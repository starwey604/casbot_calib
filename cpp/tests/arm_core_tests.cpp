#include "casbot_calib/arm_dynamics.hpp"
#include "casbot_calib/integration.hpp"
#include <Eigen/Eigenvalues>
#include <cmath>
#include <iostream>
#include <stdexcept>
using namespace casbot;
void check(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
int main() {
  // Independent planar Lagrange formula: rods in XY, gravity points -Y.
  ArmModel model;model.model_id="analytic_2r";model.gravity_world={0.,-9.81,0.};
  for(int j=0;j<2;++j) {
    ArmJoint joint;joint.name="joint_"+std::to_string(j);joint.link_name="link_"+std::to_string(j);
    joint.origin.translation={j==0?0.:.7,0.,0.};joint.body.mass=j==0?1.3:.8;
    joint.body.com={j==0?.35:.225,0.,0.};
    const double inertia=j==0?.054:.0135;joint.body.inertia_com=inertia*Eigen::Matrix3d::Identity();
    model.joints.push_back(joint);
  }
  ArmDynamics engine(model);Eigen::Vector2d q(.4,-.7),v(.8,-.5),a(-.2,.6);
  const double l=.7,c1=.35,c2=.225,m1=1.3,m2=.8,k=m2*l*c2;
  Eigen::Matrix2d expected;
  expected<<.054+.0135+m1*c1*c1+m2*(l*l+c2*c2+2*l*c2*std::cos(q[1])),.0135+m2*(c2*c2+l*c2*std::cos(q[1])),
            .0135+m2*(c2*c2+l*c2*std::cos(q[1])),.0135+m2*c2*c2;
  Eigen::Vector2d h(-k*std::sin(q[1])*(2*v[0]*v[1]+v[1]*v[1]),k*std::sin(q[1])*v[0]*v[0]);
  Eigen::Vector2d g(9.81*((m1*c1+m2*l)*std::cos(q[0])+m2*c2*std::cos(q.sum())),9.81*m2*c2*std::cos(q.sum()));
  check((engine.mass_matrix(q)-expected).norm()<1e-12,"analytic 2R mass matrix");
  check((engine.velocity_bias(q,v)-h).norm()<1e-12,"analytic 2R velocity coupling");
  check((engine.gravity(q)-g).norm()<1e-12,"analytic 2R gravity");
  check((engine.inverse(q,v,a)-(expected*a+h+g)).norm()<1e-12,"analytic 2R inverse dynamics");
  check((engine.forward(q,v,expected*a+h+g)-a).norm()<1e-11,"independent torque -> forward acceleration");
  check(std::abs(engine.energy(q,v).kinetic-.5*v.dot(expected*v))<1e-12,"physical kinetic energy");
  model.joints[0].rotor_inertia=.07;model.joints[1].rotor_inertia=.03;
  ArmDynamics rotor(model);Eigen::Matrix2d delta=Eigen::Vector2d(.07,.03).asDiagonal();
  check((rotor.mass_matrix(q)-expected-delta).norm()<1e-12,"reflected rotor included exactly once");
  auto runtime=rotor.workspace();auto second=engine.workspace();
  const auto parameters=rotor.rigid_parameters();
  check(parameters.size()==20 && rotor.rigid_parameter_names().size()==20,"body parameter schema");
  const Eigen::Vector2d torque=(expected+delta)*a+h+g;
  for(int iteration=0;iteration<8;++iteration) {
    check((runtime.torque_regressor(q,v,a)*parameters+runtime.rotor_torque(a)-torque).norm()<1e-12,
          "regressor excludes rotor and recovers independent planar torque");
    const auto di=runtime.inverse_derivatives(q,v,a);
    const auto df=runtime.forward_derivatives(q,v,torque);
    check((di.dtau_da-expected-delta).norm()<1e-12,"RNEA acceleration derivative includes armature");
    check((df.ddq_dtau*di.dtau_da-Eigen::Matrix2d::Identity()).norm()<1e-12,"ABA input derivative");
    check((df.ddq_dq+df.ddq_dtau*di.dtau_dq).norm()<1e-11,"inverse/forward state derivative consistency");
    check((runtime.components(q,v,a).total-torque).norm()<1e-12,"interleaved reusable Data consistency");
    check((second.mass_matrix(q)-expected).norm()<1e-12,"separate workspaces retain their own model");
  }
  // Tool is at the last joint origin in this fixture; its position is independent of q[1].
  Eigen::Matrix<double,6,2> jacobian=Eigen::Matrix<double,6,2>::Zero();
  jacobian(0,0)=-l*std::sin(q[0]);jacobian(1,0)=l*std::cos(q[0]);jacobian.row(5).setOnes();
  check((runtime.tool_jacobian(q)-jacobian).norm()<1e-12,"world-aligned TCP linear/angular Jacobian");
  // The same dynamic-vector Odeint backend integrates coupled Pinocchio dynamics.
  auto free_model=model;free_model.gravity_world.setZero();ArmDynamics free_arm(free_model);
  const auto integrate_arm=[&](std::int64_t step_ns) {
    Eigen::VectorXd state(4);state<<q,v;auto dynamics=free_arm.workspace();
    FixedStepIntegrator<Eigen::VectorXd> integrator(state,step_ns);
    const double initial_energy=dynamics.energy(q,v).kinetic;
    const auto rhs=[&](const Eigen::VectorXd& x,Eigen::VectorXd& rate,double) {
      rate.head(2)=x.tail(2);
      rate.tail(2)=dynamics.forward(x.head(2),x.tail(2),Eigen::Vector2d::Zero());
    };
    for(std::int64_t time=0;time<1000000000;time+=step_ns) {
      integrator.step(rhs,state,time,time+step_ns);
      check(std::abs(dynamics.energy(state.head(2),state.tail(2)).kinetic-initial_energy)<1e-9,
            "Boost with coupled Pinocchio dynamics conserves energy");
    }
    return state;
  };
  check((integrate_arm(2000000)-integrate_arm(1000000)).norm()<1e-8,"coupled Pinocchio integration step convergence");
  bool rejected=false;
  try {model.joints[0].axis*=2.;ArmDynamics invalid(model);}catch(const std::invalid_argument&){rejected=true;}
  check(rejected,"reject non-unit joint axis");
  std::cout<<"Independent planar dynamics, regressor, derivatives, Jacobian and workspace checks passed\n";
}
