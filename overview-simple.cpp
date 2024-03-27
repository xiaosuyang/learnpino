#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/fwd.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include <pinocchio/algorithm/crba.hpp>
#include "pinocchio/algorithm/dynamics.hpp"
#include <pinocchio/algorithm/frames.hpp>

#include "robotwrapper/robotwrapper.h"

#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "./model/ur5_robot.urdf"
#endif
 
int main()
{
  namespace pino=pinocchio;
  const std::string urdf_filename=PINOCCHIO_MODEL_DIR;

  pino::Model model;
  pino::urdf::buildModel(urdf_filename,model,false);
  std::cout << "model name: " << model.name << std::endl;
  std::cout<<"model.nq(Dimension of the configuration vector representation.): "<<model.nq<<'\n';
  std::cout<<"model.nv(Dimension of the velocity vector space.): "<<model.nv<<'\n';

  RobotWrapper robot(urdf_filename);

  pino::Data data(robot.m_model);
  Eigen::VectorXd q=Eigen::VectorXd::Zero(model.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(model.nv);
  q[1]=M_PI;
  v[1]=1;

  robot.computeAllTerms(data,q,v);

  std::cout<<"机器人质心位置:\n"<<robot.com(data)<<'\n';
  std::cout<<"质心速度:\n"<<robot.com_vel(data)<<'\n';
   std::cout<<"质心加速度:\n"<<robot.com_acc(data)<<'\n';


  pino::SE3 se3ins, se3in1;
  se3ins.setIdentity();
  std::cout<<"SE3 实例：\n"<<se3ins<<'\n';
  se3ins.rotation()=Eigen::Matrix3d::Identity();
  se3ins.translation()=Eigen::Vector3d(0,0,1);
  se3in1.rotation()=Eigen::Matrix3d::Identity();
  se3in1.translation()=Eigen::Vector3d(0,0,2);
  std::cout<<"After act:\n"<<se3ins.actInv(se3in1);


//  std::cout << "frame size " << model.frames.size() << std::endl;
//   for(int i=0;i<model.frames.size();i++)
//   {
//     std::cout<<"frame "<<i<<'\n'<<model.frames[i].name<<std::endl;
//     std::cout<<"parent frame:\n"<<model.frames[model.frames[i].previousFrame].name<<'\n';
//     std::cout<<"parent join:\n"<<model.names[model.frames[i].parent]<<'\n';
//     std::cout<<"----------\n";
//     std::cout<<"相对parentjoint位姿:\n"<<model.frames[i].placement<<std::endl;
  
    
//   }

  

  

//   pino::crba(model,data,q);

  //pino::Motion classica=robot.frameClassicAcceleration(data,9);

  //std::cout<<classica;
  //pino::Model::Index a;
  
  // using namespace pino;
  //  for(JointIndex joint_id = 0; joint_id < (JointIndex)model.njoints; ++joint_id)
  //   std::cout << std::setw(24) << std::left
  //             << model.names[joint_id] << ": "
  //             << std::fixed << std::setprecision(3)
  //             << data.oMi[joint_id]<<'\n'<<data.oMi[joint_id].translation().transpose()<<'\n'
  //             <<"Jdotqdot:\n"<<data.a[joint_id]<<'\n'
  //             <<"vel of joint at centers of joints:\n"<<data.v[joint_id]<<'\n'
  //             <<"vel of joint at the origin \n"<<data.ov[joint_id]
  //             << std::endl;
    
  //   std::cout<<"Joint space 惯性矩阵"<<'\n'<<data.M<<std::endl;
            


}