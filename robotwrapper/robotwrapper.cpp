#include "robotwrapper.h"
#include <iostream>

RobotWrapper::RobotWrapper(const std::string &filename, bool verbose)
    : m_verbose(verbose)
{
  pinocchio::urdf::buildModel(filename, m_model, m_verbose);
  m_model_filename = filename;
  m_na = m_model.nv;
  m_nq_actuated = m_model.nq;
  m_is_fixed_base = true;
  init();
}

RobotWrapper::RobotWrapper(const std::string &filename, const pinocchio::JointModelVariant &rootJoint,
                           bool verbose)
    : m_verbose(verbose)
{
  pinocchio::urdf::buildModel(filename, rootJoint, m_model, m_verbose);
  m_model_filename = filename;
  m_na = m_model.nv - 6;
  m_nq_actuated = m_model.nq - 7;
  m_is_fixed_base = false;
  init();
}

pino::Motion RobotWrapper::frameClassicAcceleration(const pinocchio::Data &data,
                                                    const pinocchio::Model::FrameIndex index) const
{
  using namespace pinocchio;
  PINOCCHIO_CHECK_INPUT_ARGUMENT(index < m_model.frames.size(),
                                 "Frame index greater than size of frame "
                                 "vector in model - frame may not exist");
  const Frame &f = m_model.frames[index];
  Motion a = f.placement.actInv(data.a[f.parent]);
  Motion v = f.placement.actInv(data.v[f.parent]);

  a.linear() += v.angular().cross(v.linear());
  return a;
}

void RobotWrapper::init()
{
  m_M.setZero(m_model.nv, m_model.nv);
}

void RobotWrapper::computeAllTerms(Data &data, const Eigen::VectorXd &q,
                                   const Eigen::VectorXd &v) const
{
  pinocchio::computeAllTerms(m_model, data, q, v);
  data.M.triangularView<Eigen::StrictlyLower>() =data.M.transpose().triangularView<Eigen::StrictlyLower>();
  pinocchio::updateFramePlacements(m_model, data);
  pinocchio::centerOfMass(m_model, data, q, v, Eigen::VectorXd::Zero(m_model.nv));

}

const math::Vector3& RobotWrapper::com(const Data& data) const { return data.com[0]; }

const math::Vector3& RobotWrapper::com_vel(const Data& data) const {
  return data.vcom[0];
}

const math::Vector3& RobotWrapper::com_acc(const Data& data) const {
  return data.acom[0];
}

const pino::SE3& RobotWrapper::position(const Data& data,
                                  const Model::JointIndex index) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      index < data.oMi.size(),
      "The index needs to be less than the size of the oMi vector");
  return data.oMi[index];
}

const pino::Motion& RobotWrapper::velocity(const Data& data,
                                     const Model::JointIndex index) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      index < data.v.size(),
      "The index needs to be less than the size of the v vector");
  return data.v[index];
}

const pino::Motion& RobotWrapper::acceleration(const Data& data,
                                         const Model::JointIndex index) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      index < data.a.size(),
      "The index needs to be less than the size of the a vector");
  return data.a[index];
}
