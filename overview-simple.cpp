#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/rnea.hpp"
#include "pinocchio/fwd.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include <pinocchio/algorithm/crba.hpp>
#include "pinocchio/algorithm/dynamics.hpp"
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/math/rpy.hpp>
#include <pinocchio/math/quaternion.hpp>

#include "robotwrapper/robotwrapper.h"

#ifndef PINOCCHIO_MODEL_DIR
  #define PINOCCHIO_MODEL_DIR "model/ur5_robot.urdf"
  #define PINOCCHIO_BIPED "model/biped.urdf"
#endif
 
int main()
{
  namespace pino=pinocchio;
  const std::string urdf_filename=PINOCCHIO_BIPED;
  
  pino::JointModelFreeFlyer root_joint;
  RobotWrapper robot(urdf_filename,root_joint);

  pino::Data data(robot.m_model);
  Eigen::VectorXd q=Eigen::VectorXd::Zero(robot.m_model.nq);
  Eigen::VectorXd v = Eigen::VectorXd::Zero(robot.m_model.nv);
  

  auto frpy=RobotWrapper::rpytoquat(.0,.0,M_PI);
  q.block<4,1>(3,0)=frpy;
  q[1]=2.5;


  std::cout<<"q:  "<<q<<'\n';

  robot.computeAllTerms(data,q,v);

  auto frame=robot.m_model.getFrameId(std::string("root_joint"));
  auto joint=robot.m_model.getJointId("r_j3");
  auto se3=robot.framePosition(data,frame);
  std::cout<<" frame position"<<se3<<std::endl;

  std::cout<<"joint placement:\n"<<data.oMi[joint]<<'\n';

  pino::Data::Matrix6x LJ=pino::Data::Matrix6x::Zero(6,18);
  pino::Data::Matrix6x WJ=pino::Data::Matrix6x::Zero(6,18);
  pino::Data::Matrix6x LWJ=pino::Data::Matrix6x::Zero(6,18);
  robot.jacobianLocal(data,joint,LJ);
  robot.jacobianWorld(data,joint,WJ);
  robot.jacobianWorld_Aligned(data,joint,LWJ);

  std::cout<<"World jocobian:\n"<<WJ<<'\n';

  std::cout<<"LOCAL jocobian:\n"<<LJ<<'\n';

  std::cout<<"LOCAL aligned jocobian:\n"<<LWJ<<'\n';

  
  


  
 
 

  

  // pino::SE3 se3ins, se3in1;
  // se3ins.setIdentity();
  // std::cout<<"SE3 实例：\n"<<se3ins<<'\n';
  // se3ins.rotation()=Eigen::Matrix3d::Identity();
  // se3ins.translation()=Eigen::Vector3d(0,0,1);
  // se3in1.rotation()=Eigen::Matrix3d::Identity();
  // se3in1.translation()=Eigen::Vector3d(0,0,2);
  // std::cout<<"After act:\n"<<se3ins.actInv(se3in1);
  // pino::Data::Matrix6x J;





//  std::cout << "frame size " << robot.m_model.frames.size() << std::endl;
//   for(int i=0;i<robot.m_model.frames.size();i++)
//   {
//     std::cout<<"frame "<<i<<'\n'<<robot.m_model.frames[i].name<<std::endl;
//     std::cout<<"parent frame:\n"<<robot.m_model.frames[robot.m_model.frames[i].previousFrame].name<<'\n';
//     std::cout<<"parent join:\n"<<robot.m_model.names[robot.m_model.frames[i].parent]<<'\n';
//     std::cout<<"----------\n";
//     std::cout<<"相对parentjoint位姿:\n"<<robot.m_model.frames[i].placement<<std::endl;
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