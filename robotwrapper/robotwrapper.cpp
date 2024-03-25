#include "robotwrapper.h"
#include <iostream>

RobotWrapper::RobotWrapper(const std::string &filename,
                           const std::vector<std::string> &, bool verbose)
    : m_verbose(verbose)
{
  pinocchio::urdf::buildModel(filename, m_model, m_verbose);
  m_model_filename = filename;
  m_na = m_model.nv;
  m_nq_actuated = m_model.nq;
  m_is_fixed_base = true;
  init();
}

RobotWrapper::RobotWrapper(const std::string &filename,
                           const std::vector<std::string> &,
                           const pinocchio::JointModelVariant &rootJoint,
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
  data.M.triangularView<Eigen::StrictlyLower>() =
      data.M.transpose().triangularView<Eigen::StrictlyLower>();
    

}
