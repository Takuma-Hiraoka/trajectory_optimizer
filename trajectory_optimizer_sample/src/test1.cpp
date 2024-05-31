#include <trajectory_optimizer/trajectory_optimizer.h>
#include <ik_constraint2/ik_constraint2.h>
#include <cnoid/BodyLoader>
#include <ros/package.h>
#include <random>

int main(void){
  // load robot
  std::string modelfile = ros::package::getPath("choreonoid") + "/share/model/SR1/SR1.body";
  cnoid::BodyLoader bodyLoader;
  cnoid::BodyPtr robot = bodyLoader.load(modelfile);
  robot->link("RARM_ELBOW")->setJointRange(-150.0/180.0*M_PI, 90.0/180.0*M_PI);
  robot->link("LARM_ELBOW")->setJointRange(-150.0/180.0*M_PI, 90.0/180.0*M_PI);
  robot->link("RLEG_KNEE")->setJointRange(-90.0/180.0*M_PI, 150.0/180.0*M_PI);
  robot->link("LLEG_KNEE")->setJointRange(-90.0/180.0*M_PI, 150.0/180.0*M_PI);

  std::vector<double> init_q(robot->numJoints()+7);
  init_q[3] = 0;
  init_q[4] = 0;
  init_q[5] = 0;
  init_q[6] = 1;
  std::vector<double> end_q(robot->numJoints()+7, 1);
  end_q[3] = 0;
  end_q[4] = 0;
  end_q[5] = 0;
  end_q[6] = 1;
  std::shared_ptr<std::vector<std::vector<double> > > path = std::make_shared<std::vector<std::vector<double> > >();
  path->push_back(init_q);
  path->push_back(init_q);
  path->push_back(init_q);
  path->push_back(end_q);
  path->push_back(end_q);

  std::vector<cnoid::LinkPtr> variables;
  variables.push_back(robot->rootLink());
  for(size_t i=0;i<robot->numJoints();i++){
    variables.push_back(robot->joint(i));
  }

  // setup constraints
  std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > constraints0;
  // joint limit
  for(int i=0;i<robot->numJoints();i++){
    std::shared_ptr<ik_constraint2::JointLimitConstraint> constraint = std::make_shared<ik_constraint2::JointLimitConstraint>();
    constraint->joint() = robot->joint(i);
    constraints0.push_back(constraint);
  }
  std::vector<std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > > constraints{constraints0};
  std::vector<std::shared_ptr<ik_constraint2::IKConstraint> > rejections;
  trajectory_optimizer::TOParam param;
  std::cerr << "initial path" << std::endl;
  for (int i=0;i<path->size();i++) {
    for (int j=0;j<(*path)[i].size();j++) std::cerr << (*path)[i][j] << " ";
    std::cerr << std::endl;
  }
  bool solved = trajectory_optimizer::solveTO(variables,
                                              constraints,
                                              rejections,
                                              param,
                                              path);
  std::cerr << "solved: " << solved << std::endl;
  std::cerr << "end path" << std::endl;
  for (int i=0;i<path->size();i++) {
    for (int j=0;j<(*path)[i].size();j++) std::cerr << (*path)[i][j] << " ";
    std::cerr << std::endl;
  }

  return 0;
}
