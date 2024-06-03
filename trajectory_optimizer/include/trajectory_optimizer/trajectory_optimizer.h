#ifndef TRAJECTORY_OPTIMIZER_H
#define TRAJECTORY_OPTIMIZER_H

#include <ik_constraint2/ik_constraint2.h>
#include <prioritized_inverse_kinematics_solver2/prioritized_inverse_kinematics_solver2.h>
#include <thread>
#include <mutex>

namespace trajectory_optimizer{
  class TOParam {
  public:
    int debugLevel = 0; // 0: no debug message. 1: time measure. 2: internal state
    int maxIteration = 100;
    double convergeThre = 1e-2;
    prioritized_inverse_kinematics_solver2::IKParam pikParam;
    unsigned int threads = 1;
    std::shared_ptr<std::mutex> threadLock = std::make_shared<std::mutex>();

    TOParam(){
      pikParam.we = 1e2;
      pikParam.maxIteration = 10;
      pikParam.minIteration = 1;
      pikParam.checkFinalState = true;
      pikParam.calcVelocity = false;
      pikParam.convergeThre = 2.5e-1;
    }
  };
  bool solveTOOnce(const std::vector<std::vector<cnoid::LinkPtr> >& variables,
                   const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints,
                   const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& rejections,
                   prioritized_inverse_kinematics_solver2::IKParam pikParam);
  void solveTOThread(std::shared_ptr<unsigned int> count,
		     std::shared_ptr<std::mutex> mutex,
		     const std::vector<std::vector<cnoid::LinkPtr> >& variables,
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
