#include <trajectory_optimizer/trajectory_optimizer.h>

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

  bool solveTO(const std::vector<cnoid::LinkPtr>& variables,
               const std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > >& constraints,
               const std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& rejections,
               const TOParam& param,
               std::shared_ptr<std::vector<std::vector<double> > > path){
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

    int loop;
    for (loop=0; loop < param.maxIteration; loop++){

      // add trajectory constraint
      for (int i=1; i< path->size() -1; i++){
        std::vector<std::shared_ptr<ik_constraint2::IKConstraint> >& trajectoryConstraints = constraintss[i].back();
        trajectoryConstraints.clear(); // 前回loop時のものを取り除く
        for(int v=0;v<variabless[i].size();v++){
          if(variabless[i][v]->isRevoluteJoint() || variabless[i][v]->isPrismaticJoint()){
            std::shared_ptr<ik_constraint2::JointAngleConstraint> forwardConstraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
            forwardConstraint->joint() = variabless[i][v];
            forwardConstraint->targetq() = variabless[i-1][v]->q();
            forwardConstraint->precision() = 1e10; // always satisfied
            forwardConstraint->maxError() = 1e10;
            trajectoryConstraints.push_back(forwardConstraint);

            std::shared_ptr<ik_constraint2::JointAngleConstraint> backwardConstraint = std::make_shared<ik_constraint2::JointAngleConstraint>();
            backwardConstraint->joint() = variabless[i][v];
            backwardConstraint->targetq() = variabless[i+1][v]->q();
            backwardConstraint->precision() = 1e10; // always satisfied
            backwardConstraint->maxError() = 1e10;
            trajectoryConstraints.push_back(backwardConstraint);

          }else if(variabless[i][v]->isFreeJoint()) {
            std::shared_ptr<ik_constraint2::PositionConstraint> forwardConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
            forwardConstraint->A_link() = variabless[i][v];
            forwardConstraint->B_localpos() = variabless[i-1][v]->T();
            forwardConstraint->precision() = 1e10; // always satisfied
            forwardConstraint->maxError() << 1e10, 1e10, 1e10, 1e10, 1e10, 1e10;
            trajectoryConstraints.push_back(forwardConstraint);

            std::shared_ptr<ik_constraint2::PositionConstraint> backwardConstraint = std::make_shared<ik_constraint2::PositionConstraint>();
            backwardConstraint->A_link() = variabless[i][v];
            backwardConstraint->B_localpos() = variabless[i+1][v]->T();
            backwardConstraint->precision() = 1e10; // always satisfied
            backwardConstraint->maxError() << 1e10, 1e10, 1e10, 1e10, 1e10, 1e10;
            trajectoryConstraints.push_back(backwardConstraint);
          }else{
            std::cerr << "[trajectory_optimizer::solveTO] something is wrong" << std::endl;
          }
        }
      }// add trajectory constraint

      bool solved = solveTOOnce(variabless, constraintss, rejectionss, param.pikParam);
    }
    for (int i=0; i<path->size();i++) {
      link2Frame(variabless[i], (*path)[i]);
    }
    return true;
  }
}
