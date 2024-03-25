
#ifndef robot_wrapper_hpp__
#define robot_wrapper_hpp__

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include "pinocchio/parsers/urdf.hpp"

namespace pino = pinocchio;

class RobotWrapper
{
public:
  typedef pinocchio::Model Model;
  typedef pinocchio::Data Data;
  typedef pinocchio::Motion Motion;
  typedef pinocchio::Frame Frame;
  typedef double Scalar;
  Motion frameClassicAcceleration(const Data &data,
                                  const Model::FrameIndex index) const;
  RobotWrapper(const std::string &filename,
               const std::vector<std::string> &package_dirs,
               bool verbose = false);

  RobotWrapper(const std::string &filename,
               const std::vector<std::string> &package_dirs,
               const pinocchio::JointModelVariant &rootJoint,
               bool verbose = false);

  RobotWrapper(std::string modelname)
  {
    m_model_filename = modelname;
    pino::urdf::buildModel(m_model_filename, m_model);
  }
  void init();

  pino::Model m_model;
  std::string m_model_filename;
  int m_nq_actuated; /// dimension of the configuration space of the actuated
                     /// DoF (nq for fixed-based, nq-7 for floating-base
                     /// robots)
  int m_na;          /// number of actuators (nv for fixed-based, nv-6 for floating-base
                     /// robots)
  bool m_is_fixed_base;
  bool m_verbose;
  void computeAllTerms(Data& data, const Eigen::VectorXd& q,
                                   const Eigen::VectorXd& v) const;
  Eigen::Matrix<Scalar,Eigen::Dynamic,Eigen::Dynamic> m_M;
};

#endif
