#include <trajectory_optimizer/trajectory_optimizer.h>
#include <cnoid/TimeMeasure>

namespace trajectory_optimizer{

  inline std::set<cnoid::BodyPtr> getBodies(const std::vector<cnoid::LinkPtr>& links){
    std::set<cnoid::BodyPtr> bodies;
    for(size_t i=0;i<links.size();i++){
      if(links[i]->body()) bodies.insert(links[i]->body());
    }
    return bodies;
  }

  inline void link2Frame(const std::vector<cnoid::LinkPtr>& links, std::vector<double>& frame){
    frame.clear();
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        frame.push_back(links[l]->q());
      }else if(links[l]->isFreeJoint()) {
        frame.push_back(links[l]->p()[0]);
        frame.push_back(links[l]->p()[1]);
        frame.push_back(links[l]->p()[2]);
        cnoid::Quaternion q(links[l]->R());
        frame.push_back(q.x());
        frame.push_back(q.y());
        frame.push_back(q.z());
        frame.push_back(q.w());
      }
    }
  }

  inline void frame2Link(std::vector<double>& frame, const std::vector<cnoid::LinkPtr>& links){
    int idx = 0;
    for(int l=0;l<links.size();l++){
      if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
        links[l]->q() = frame[idx];
        idx++;
      }else if(links[l]->isFreeJoint()) {
        links[l]->p()[0] = frame[idx+0];
        links[l]->p()[1] = frame[idx+1];
        links[l]->p()[2] = frame[idx+2];
        cnoid::Quaternion q(frame[idx+6],frame[idx+3],frame[idx+4],frame[idx+5]);
        links[l]->R() = q.toRotationMatrix();
        idx+=7;
      }
    }
  }

  inline std::vector<unsigned int> shortcuttableIdx(std::shared_ptr<std::vector<std::vector<double> > > path, const std::vector<cnoid::LinkPtr>& links, const double thre){
    std::vector<unsigned int> idxs;
    for(int i=1;i<path->size()-1;i++){
      unsigned int idx = 0;
      bool cut = true;
      for(int l=0;l<links.size();l++){
        if(links[l]->isRevoluteJoint() || links[l]->isPrismaticJoint()) {
          if ((std::abs((*path)[i][idx] - (*path)[i-1][idx]) > thre) ||
              (std::abs((*path)[i][idx] - (*path)[i-1][idx]) > thre)) {
            cut = false;
            break;
          }
          idx++;
        } else if(links[l]->isFreeJoint()) {
          cnoid::Quaternion prevQ((*path)[i-1][idx+6], (*path)[i-1][idx+3], (*path)[i-1][idx+4], (*path)[i-1][idx+5]);
          cnoid::Quaternion q((*path)[i][idx+6], (*path)[i][idx+3], (*path)[i][idx+4], (*path)[i][idx+5]);
          cnoid::Quaternion nextQ((*path)[i+1][idx+6], (*path)[i+1][idx+3], (*path)[i+1][idx+4], (*path)[i+1][idx+5]);
          cnoid::Matrix3 prevR = prevQ.toRotationMatrix();
          cnoid::Matrix3 R = q.toRotationMatrix();
          cnoid::Matrix3 nextR = nextQ.toRotationMatrix();
          cnoid::AngleAxis prevAngleAxis = cnoid::AngleAxis(R * prevR.transpose());
          cnoid::Vector3 prev = prevAngleAxis.angle()*prevAngleAxis.axis();
          cnoid::AngleAxis nextAngleAxis = cnoid::AngleAxis(nextR * R.transpose());
          cnoid::Vector3 next = nextAngleAxis.angle()*nextAngleAxis.axis();
          if ((std::abs((*path)[i][idx+0] - (*path)[i-1][idx+0]) > thre) ||
              (std::abs((*path)[i][idx+1] - (*path)[i-1][idx+1]) > thre) ||
              (std::abs((*path)[i][idx+2] - (*path)[i-1][idx+2]) > thre) ||
              (std::abs(prev[0]) > thre) ||
              (std::abs(prev[1]) > thre) ||
              (std::abs(prev[2]) > thre) ||
              (std::abs((*path)[i][idx+0] - (*path)[i+1][idx+0]) > thre) ||
              (std::abs((*path)[i][idx+1] - (*path)[i+1][idx+1]) > thre) ||
              (std::abs((*path)[i][idx+2] - (*path)[i+1][idx+2]) > thre) ||
              (std::abs(prev[0]) > thre) ||
              (std::abs(prev[1]) > thre) ||
              (std::abs(prev[2]) > thre)) {
            cut = false;
            break;
          }
          idx+=7;
        }
      }
      if (cut) {
        idxs.push_back(i);
        i++; //連続stateのcutはしない
      }
    }
    return idxs;
  }

  bool solveTOOnce(const std::vector<std::vector<cnoid::LinkPtr> >& variables,
                   const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints,
                   const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& rejections,
                   prioritized_inverse_kinematics_solver2::IKParam pikParam){
    for (int i=0;i<variables.size();i++){
      std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
      std::shared_ptr<std::vector<std::vector<double> > > path;
      bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variables[i],
                                                                        constraints[i],
                                                                        rejections[i],
                                                                        tasks,
                                                                        pikParam,
                                                                        path);
    }
    return true;
  }


  void solveTOThread(std::shared_ptr<unsigned int> count,
                     std::shared_ptr<std::mutex> mutex,
                     const std::vector<std::vector<cnoid::LinkPtr> >& variables,
                     const std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > >& constraints,
                     const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& rejections,
                     prioritized_inverse_kinematics_solver2::IKParam pikParam) {
    while (true) {
      mutex->lock();
      if ((*count) >= variables.size()) {
        mutex->unlock();
        return;
      } else {
        std::vector<cnoid::LinkPtr> variable = variables[*count];
        std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraint = constraints[*count];
        std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejection = rejections[*count];
        (*count)++;
        mutex->unlock();
        std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
        std::shared_ptr<std::vector<std::vector<double> > > path;
        bool solved =  prioritized_inverse_kinematics_solver2::solveIKLoop(variable,
                                                                           constraint,
                                                                           rejection,
                                                                           tasks,
                                                                           pikParam,
                                                                           path);
      }
    }
  }

  bool solveTOParallel(const std::vector<cnoid::LinkPtr>& variables,
                       const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
                       const TOParam& param,
                       std::shared_ptr<std::vector<std::vector<double> > > path){
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejections;
    return solveTOParallel(variables, constraints, rejections, param, path);
  }

  bool solveTOParallel(const std::vector<cnoid::LinkPtr>& variables,
                       const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
                       const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections,
                       const TOParam& param,
                       std::shared_ptr<std::vector<std::vector<double> > > path){
    if (path->size()<3) return true;

    if(param.debugLevel > 1) {
      std::cerr << "[TrajectoryOptimizer] input path size : " << path->size() << std::endl;
    }

    if (param.shortcut) {
      std::vector<unsigned int> idxs = shortcuttableIdx(path, variables, param.shortcutThre);
      int cutNum = 0;
      for (int i=0;i<idxs.size();i++) {
        path->erase(path->begin() + idxs[i] - cutNum);
        cutNum++;
      }
      if(param.debugLevel > 1) {
        std::cerr << "[TrajectoryOptimizer] initial cut : " << idxs.size() << " states. current path size : "  << path->size() << std::endl;
      }
    }
    // copy
    std::vector<std::map<cnoid::BodyPtr, cnoid::BodyPtr> > modelMaps;
    std::vector<std::vector<cnoid::LinkPtr> > variabless;
    std::vector<std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > > constraintss;
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > rejectionss;
    for (int i=0; i< path->size(); i++){
      std::set<cnoid::BodyPtr> bodies = getBodies(variables);
      std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
      for(std::set<cnoid::BodyPtr>::iterator it = bodies.begin(); it != bodies.end(); it++){
        modelMap[*it] = (*it)->clone();
      }
      modelMaps.push_back(modelMap); // cloneしたbodyがデストラクトされないように、保管しておく
      variabless.push_back(std::vector<cnoid::LinkPtr>(variables.size()));
      for(int v=0;v<variables.size();v++){
        variabless.back()[v] = modelMap[variables[v]->body()]->link(variables[v]->index());
      }
      frame2Link((*path)[i],variabless.back());
      constraintss.push_back(std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >(constraints.size()));
      for(int j=0;j<constraints.size();j++){
        constraintss.back()[j].resize(constraints[j].size());
        for(int k=0;k<constraints[j].size();k++){
          constraintss.back()[j][k] = constraints[j][k]->clone(modelMap);
        }
        constraintss.back().push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >());
      }
      rejectionss.push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(rejections.size()));
      for(int j=0;j<rejections.size();j++){
        rejectionss.back()[j] = rejections[j]->clone(modelMap);
      }
    } // copy

    cnoid::TimeMeasure timer;
    if(param.debugLevel>0) timer.begin();

    std::vector<std::vector<double> > prevPath;
    int loop;
    for (loop=0; loop < param.maxIteration; loop++){

      // add trajectory constraint
      for (int i=1; i< path->size() -1; i++){
        std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& trajectoryConstraints = constraintss[i].back();
        trajectoryConstraints.clear(); // 前回loop時のものを取り除く
        for(int v=0;v<variabless[i].size();v++){
          if(variabless[i][v]->isRevoluteJoint() || variabless[i][v]->isPrismaticJoint()){
            std::shared_ptr<ik_constraint2::JointAngleConstraint> forwardConstraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
            forwardConstraint->A_joint() = variabless[i][v];
            forwardConstraint->B_q() = variabless[i-1][v]->q();
            forwardConstraint->precision() = 1e-3; // never satisfied
            forwardConstraint->maxError() = 1e10;
            trajectoryConstraints.push_back(forwardConstraint);

            std::shared_ptr<ik_constraint2::JointAngleConstraint> backwardConstraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
            backwardConstraint->A_joint() = variabless[i][v];
            backwardConstraint->B_q() = variabless[i+1][v]->q();
            backwardConstraint->precision() = 1e-3; // never satisfied
            backwardConstraint->maxError() = 1e10;
            trajectoryConstraints.push_back(backwardConstraint);

          }else if(variabless[i][v]->isFreeJoint()) {
            std::shared_ptr<ik_constraint2::PositionConstraint> forwardConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
            forwardConstraint->A_link() = variabless[i][v];
            forwardConstraint->B_localpos() = variabless[i-1][v]->T();
            forwardConstraint->precision() = 1e-3; // never satisfied
            forwardConstraint->maxError() << 1e10, 1e10, 1e10, 1e10, 1e10, 1e10;
            trajectoryConstraints.push_back(forwardConstraint);

            std::shared_ptr<ik_constraint2::PositionConstraint> backwardConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
            backwardConstraint->A_link() = variabless[i][v];
            backwardConstraint->B_localpos() = variabless[i+1][v]->T();
            backwardConstraint->precision() = 1e-3; // never satisfied
            backwardConstraint->maxError() << 1e10, 1e10, 1e10, 1e10, 1e10, 1e10;
            trajectoryConstraints.push_back(backwardConstraint);
          }else{
            std::cerr << "[trajectory_optimizer::solveTO] something is wrong" << std::endl;
          }
        }
      }// add trajectory constraint

      int numThreads = (param.threads > path->size()) ? path->size() : param.threads;

      if (numThreads > 1) {
        std::shared_ptr<unsigned int> count = std::make_shared<unsigned int>(0);
        std::vector<std::thread *> th(numThreads);
        std::shared_ptr<std::mutex> threadLock = std::make_shared<std::mutex>();
        for (unsigned int i=0; i < numThreads; i++) {
          th[i] = new std::thread([&count, &threadLock, &variabless, &constraintss, &rejectionss, &param]
                                  { return solveTOThread(count, threadLock, variabless, constraintss, rejectionss, param.pikParam);
                                  });
        }
        for (unsigned int i=0; i < numThreads; i++) {
          th[i]->join();
          delete th[i];
        }
      } else {
        bool solved = solveTOOnce(variabless, constraintss, rejectionss, param.pikParam);
      }

      // 収束判定
      prevPath= (*path);
      double distance=0;
      for(int i=0;i<path->size();i++){
        link2Frame(variabless[i], (*path)[i]); // 更新
        int idx = 0;
        for(int v=0;v<variabless[i].size();v++){
          if(variabless[i][v]->isRevoluteJoint() || variabless[i][v]->isPrismaticJoint()) {
            distance += std::abs(prevPath[i][idx] - (*path)[i][idx]);
            idx++;
          } else if(variabless[i][v]->isFreeJoint()) {
            distance += std::abs(prevPath[i][idx+0] - (*path)[i][idx+0]);
            distance += std::abs(prevPath[i][idx+1] - (*path)[i][idx+1]);
            distance += std::abs(prevPath[i][idx+2] - (*path)[i][idx+2]);
            cnoid::Quaternion prevQ(prevPath[i][idx+6], prevPath[i][idx+3], prevPath[i][idx+4], prevPath[i][idx+5]);
            cnoid::Quaternion q((*path)[i][idx+6], (*path)[i][idx+3], (*path)[i][idx+4], (*path)[i][idx+5]);
            cnoid::Matrix3 prevR = prevQ.toRotationMatrix();
            cnoid::Matrix3 R = q.toRotationMatrix();
            cnoid::AngleAxis angleAxis = cnoid::AngleAxis(R * prevR.transpose());
            distance += (angleAxis.angle()*angleAxis.axis()).sum();
            idx+=7;
          }
        }
      }

      bool converged = true;
      if (param.shortcut) {
        std::vector<unsigned int> idxs = shortcuttableIdx(path, variables, param.shortcutThre);
        int cutNum = 0;
        for (int i=0;i<idxs.size();i++) {
          modelMaps.erase(modelMaps.begin() + (idxs[i] - cutNum));
          variabless.erase(variabless.begin() + (idxs[i] - cutNum));
          constraintss.erase(constraintss.begin() + (idxs[i] - cutNum));
          rejectionss.erase(rejectionss.begin() + (idxs[i] - cutNum));
          path->erase(path->begin() + idxs[i] - cutNum);
          cutNum++;
        }
        if (idxs.size() != 0) {
          converged = false;
          if(param.debugLevel > 1) {
            std::cerr << "[TrajectoryOptimizer] cut : " << idxs.size() << " states. current path size : "  << path->size() << std::endl;
          }
        }

      }

      if ( converged && (distance <= param.convergeThre)) break;
    }
    if(param.debugLevel > 0) {
      double time = timer.measure();
      std::cerr << "[TrajectoryOptimizer] solveTO loop: " << loop << " time: " << time << "[s]." << std::endl;
    }
    return true;
  }

  bool solveTO(const std::vector<cnoid::LinkPtr>& variables,
                       const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
                       const TOParam& param,
                       std::shared_ptr<std::vector<std::vector<double> > > path){
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejections;
    return solveTO(variables, constraints, rejections, param, path);
  }

  bool solveTO(const std::vector<cnoid::LinkPtr>& variables,
               const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
               const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections,
               const TOParam& param,
               std::shared_ptr<std::vector<std::vector<double> > > path){
    if (path->size()<3) return true;

    if(param.debugLevel > 1) {
      std::cerr << "[TrajectoryOptimizer] input path size : " << path->size() << std::endl;
    }

    if (param.shortcut) {
      std::vector<unsigned int> idxs = shortcuttableIdx(path, variables, param.shortcutThre);
      int cutNum = 0;
      for (int i=0;i<idxs.size();i++) {
        path->erase(path->begin() + idxs[i] - cutNum);
        cutNum++;
      }
      if(param.debugLevel > 1) {
        std::cerr << "[TrajectoryOptimizer] initial cut : " << idxs.size() << " states. current path size : "  << path->size() << std::endl;
      }
    }
    // copy
    std::vector<std::map<cnoid::BodyPtr, cnoid::BodyPtr> > modelMaps;
    std::vector<cnoid::LinkPtr> variabless;
    std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraintss;
    constraintss.resize(constraints.size());
    std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejectionss;
    for (int i=0; i< path->size(); i++){
      std::set<cnoid::BodyPtr> bodies = getBodies(variables);
      std::map<cnoid::BodyPtr, cnoid::BodyPtr> modelMap;
      for(std::set<cnoid::BodyPtr>::iterator it = bodies.begin(); it != bodies.end(); it++){
        modelMap[*it] = (*it)->clone();
      }
      modelMaps.push_back(modelMap); // cloneしたbodyがデストラクトされないように、保管しておく
      for(int v=0;v<variables.size();v++){
        variabless.push_back(modelMap[variables[v]->body()]->link(variables[v]->index()));
      }
      frame2Link((*path)[i],std::vector<cnoid::LinkPtr>(variabless.begin() + i * variables.size(), variabless.begin() + (i+1) * variables.size()));
      for(int j=0;j<constraints.size();j++){ //優先度
        for(int k=0;k<constraints[j].size();k++){
          constraintss[j].push_back(constraints[j][k]->clone(modelMap));
        }
      }
      for(int j=0;j<rejections.size();j++){
        rejectionss.push_back(rejections[j]->clone(modelMap));
      }
    } // copy
    constraintss.push_back(std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >(constraints.size()));

    cnoid::TimeMeasure timer;
    if(param.debugLevel>0) timer.begin();

    int loop;
    for (loop=0; loop < param.maxIteration; loop++) {
      std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& trajectoryConstraints = constraintss.back();
      trajectoryConstraints.clear();
      for (int i=0; i<path->size() -1; i++) {
        for(int v=0;v<variables.size();v++) { //variablessの順番はvariablesと同じ
          if(variabless[v]->isRevoluteJoint() || variabless[v]->isPrismaticJoint()){
            std::shared_ptr<ik_constraint2::JointAngleConstraint> backwardConstraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
            if (i==0) {
              backwardConstraint->A_q() = variabless[v + i*variables.size()]->q();
            } else {
              backwardConstraint->A_joint() = variabless[v + i*variables.size()];
            }
            if (i==(path->size() -2)) {
              backwardConstraint->B_q() = variabless[v + (i+1)*variables.size()]->q();
            } else {
              backwardConstraint->B_joint() = variabless[v + (i+1)*variables.size()];
            }
            backwardConstraint->precision() = 1e-3; // never satisfied
            backwardConstraint->maxError() = 1e10;
            trajectoryConstraints.push_back(backwardConstraint);

          }else if(variabless[v]->isFreeJoint()) {
            std::shared_ptr<ik_constraint2::PositionConstraint> backwardConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
            if (i==0) {
              backwardConstraint->A_localpos() = variabless[v + i*variables.size()]->T();
            } else {
              backwardConstraint->A_link() = variabless[v + i*variables.size()];
            }
            if (i==(path->size() -2)) {
              backwardConstraint->B_localpos() = variabless[v + (i+1)*variables.size()]->T();
            } else {
              backwardConstraint->B_link() = variabless[v + (i+1)*variables.size()];
            }
            backwardConstraint->precision() = 1e-3; // never satisfied
            backwardConstraint->maxError() << 1e10, 1e10, 1e10, 1e10, 1e10, 1e10;
            trajectoryConstraints.push_back(backwardConstraint);
          }else{
            std::cerr << "[trajectory_optimizer::solveTO] something is wrong" << std::endl;
          }
        }
      }
      // solve
      std::vector<std::shared_ptr<prioritized_qp_base::Task> > tasks;
      std::shared_ptr<std::vector<std::vector<double> > > pikPath;
      bool solved = prioritized_inverse_kinematics_solver2::solveIKLoop(variabless,
                                                                        constraintss,
                                                                        rejectionss,
                                                                        tasks,
                                                                        param.pikParam,
                                                                        pikPath);
      for(int i=0;i<path->size();i++){
        link2Frame(std::vector<cnoid::LinkPtr>(variabless.begin() + i * variables.size(), variabless.begin() + (i+1) * variables.size()), (*path)[i]); // 更新
      }

      bool converged = true;
      if (param.shortcut) {
        std::vector<unsigned int> idxs = shortcuttableIdx(path, variables, param.shortcutThre);
        int cutNum = 0;
        for (int i=0;i<idxs.size();i++) {
          modelMaps.erase(modelMaps.begin() + (idxs[i] - cutNum));
          variabless.erase(variabless.begin() + (idxs[i] - cutNum)*variables.size(), variabless.begin() + (idxs[i] - cutNum+1)*variables.size());
          for (int c=0;c<constraints.size();c++) {
            constraintss[c].erase(constraintss[c].begin() + (idxs[i] - cutNum)*constraints[c].size(), constraintss[c].begin() + (idxs[i] - cutNum+1)*constraints[c].size());
          }
          rejectionss.erase(rejectionss.begin() + (idxs[i] - cutNum)*rejections.size(), rejectionss.begin() + (idxs[i] - cutNum+1)*rejections.size());
          path->erase(path->begin() + idxs[i] - cutNum);
          cutNum++;
        }
        if (idxs.size() != 0) {
          converged = false;
          if(param.debugLevel > 1) {
            std::cerr << "[TrajectoryOptimizer] cut : " << idxs.size() << " states. current path size : "  << path->size() << std::endl;
          }
        }
      }

      if (converged) break;
    }
    if(param.debugLevel > 0) {
      double time = timer.measure();
      std::cerr << "[TrajectoryOptimizer] solveTO loop: " << loop << " time: " << time << "[s]." << std::endl;
    }
    if(param.debugLevel > 1) {
      std::cerr << "[TrajectoryOptimizer] output path size : " << path->size() << std::endl;
    }
    return true;
  }
}
