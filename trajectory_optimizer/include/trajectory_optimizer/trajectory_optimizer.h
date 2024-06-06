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
    double convergeThre = 1e-2; // 並列計算用. これを満たしていても、shortcutできるならする.
    prioritized_inverse_kinematics_solver2::IKParam pikParam;
    unsigned int threads = 1;
    bool initialShortcut = true; //初期軌道でshortcutを行うかどうか.
    bool shortcut = false; //ほぼ変位のないstateを取り除いてstate数を減らすかどうか. trueの場合state数が変わらなくなるまで計算を続ける.
    double shortcutThre = 1e-3; // 前後のstate間で全ての自由度の変位がこの値以下である場合そのstateを取り除く.

    TOParam(){
      pikParam.we = 1e2;
      pikParam.maxIteration = 5;
      pikParam.minIteration = 0;
      pikParam.checkFinalState = true;
      pikParam.calcVelocity = false;
      pikParam.convergeThre = 2.5e-1; // precisionを小さく取って実質satisfiedになることはほとんどないため,一度のpikで軌道最適化するときはこの値が軌道の収束判定値
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
  bool solveTOParallel(const std::vector<cnoid::LinkPtr>& variables,
		       const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
		       const TOParam& param = TOParam(),
		       std::shared_ptr<std::vector<std::vector<double> > > path = nullptr);
  bool solveTOParallel(const std::vector<cnoid::LinkPtr>& variables,
		       const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
		       const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections,
		       const TOParam& param = TOParam(),
		       std::shared_ptr<std::vector<std::vector<double> > > path = nullptr);
  bool solveTO(const std::vector<cnoid::LinkPtr>& variables,
               const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
               const TOParam& param = TOParam(),
               std::shared_ptr<std::vector<std::vector<double> > > path = nullptr);
  bool solveTO(const std::vector<cnoid::LinkPtr>& variables,
               const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
               const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections,
               const TOParam& param = TOParam(),
               std::shared_ptr<std::vector<std::vector<double> > > path = nullptr);
}

#endif
