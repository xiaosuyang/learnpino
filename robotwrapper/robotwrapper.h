
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
#include "fwd.hpp"

namespace pino = pinocchio;


class RobotWrapper
{
public:
  typedef pinocchio::Model Model;
  typedef pinocchio::Data Data;
  typedef pinocchio::Motion Motion;
  typedef pinocchio::Frame Frame;
  typedef pinocchio::SE3 SE3;
  typedef math::Vector Vector;
  typedef math::Vector3 Vector3;
  typedef math::Vector6 Vector6;
  typedef math::Matrix Matrix;
  typedef math::Matrix3x Matrix3x;
  typedef double Scalar;
  Motion frameClassicAcceleration(const Data &data,
                                  const Model::FrameIndex index) const;
  RobotWrapper(const std::string &filename,
               bool verbose = false);

  RobotWrapper(const std::string &filename,
               const pinocchio::JointModelVariant &rootJoint,
               bool verbose = false);

  void init();

  const Vector3& com(const Data& data) const;
  const Vector3& com_vel(const Data& data) const;
  const Vector3& com_acc(const Data& data) const;

  const SE3& position(const Data& data, const Model::JointIndex index) const;
  const Motion& velocity(const Data& data, const Model::JointIndex index) const;
  const Motion& acceleration(const Data& data, const Model::JointIndex index) const;

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
