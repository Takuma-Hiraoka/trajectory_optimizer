#ifndef TRAJECTORY_OPTIMIZER_H
#define TRAJECTORY_OPTIMIZER_H

#include <ik_constraint2/ik_constraint2.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>

namespace trajectory_optimizer{
  class TOParam {
  public:
    int debugLevel = 0; // 0: no debug message. 1: time measure. 2: internal state
    int maxIteration = 100;
    prioritized_inverse_kinematics_solver2::IKParam pikParam;

    TOParam(){
      pikParam.we = 1e2;
      pikParam.maxIteration = 10;
      pikParam.minIteration = 10;
      pikParam.checkFinalState = true;
      pikParam.calcVelocity = false;
      pikParam.convergeThre = 2.5e-2;
    }
  };
  bool solveTOOnce(const std::vector<std::vector<cnoid::LinkPtr> >& variables,
                   const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints,
                   const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& rejections,
                   prioritized_inverse_kinematics_solver2::IKParam pikParam);
  bool solveTO(const std::vector<cnoid::LinkPtr>& variables,
               const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
               const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections,
               const TOParam& param = TOParam(),
               std::shared_ptr<std::vector<std::vector<double> > > path = nullptr);
}

#endif
