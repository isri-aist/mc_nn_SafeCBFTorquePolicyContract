#include "policy/PolicyConfig.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>

namespace rlqp
{
//============================================================================//
// Helper function
//============================================================================//
namespace
{
double find_value(std::map<std::string, double> map, std::string joint, std::string map_name, double def = -1)
{
  auto it = map.find(joint);
  if(it == map.end() && def != -1)
  {
    mc_rtc::log::warning("[PolicyConfig] Missing entry {} in {}", joint, map_name);
    return def;
  }
  return it->second;
}

std::string basenameWithoutExtension(const std::string & path)
{
  std::string base = path;

  const size_t slash = base.find_last_of('/');
  if(slash != std::string::npos) base = base.substr(slash + 1);

  const size_t dot = base.find_last_of('.');
  if(dot != std::string::npos) base = base.substr(0, dot);

  return base;
}

std::string joinPath(const std::string & lhs, const std::string & rhs)
{
  if(lhs.empty()) return rhs;

  if(lhs.back() == '/') return lhs + rhs;

  return lhs + "/" + rhs;
}

} // namespace

//============================================================================//
// PolicyConfig
//============================================================================//
PolicyConfig PolicyConfig::load(const std::string & policyFolder, std::vector<std::string> mcRtcJoints)
{
  PolicyConfig out;

  out.folder = policyFolder;
  out.policyYamlPath = joinPath(policyFolder, "policy.yaml");
  out.observationsYamlPath = joinPath(policyFolder, "observations.yaml");

  out.policyConfiguration.load(out.policyYamlPath);
  out.observationsConfiguration.load(out.observationsYamlPath);

  int joint_size = mcRtcJoints.size();

  out.kp = std::vector<double>(joint_size);
  out.kd = std::vector<double>(joint_size);

  out.name = out.policyConfiguration("name", basenameWithoutExtension(policyFolder));

  const std::string defaultOnnxName = out.name + ".onnx";
  const std::string onnxFile = out.policyConfiguration("onnx", defaultOnnxName);
  out.onnxPath = joinPath(policyFolder, onnxFile);
  std::map<std::string, double> kp_map, kd_map, defaultPos_map, actionScale_map;

  if(out.policyConfiguration.has("control"))
  {
    const mc_rtc::Configuration control = out.policyConfiguration("control");

    out.useQP = control("use_QP", out.policyConfiguration("use_QP", true));

    const bool hasPeriod = control.has("period_s");
    const bool hasFrequency = control.has("frequency_hz");
    const bool hasCmdVelSpeed = control.has("vel_cmd_speed");
    if(hasPeriod && hasFrequency)
      mc_rtc::log::error_and_throw("[PolicyConfig:{}] Specify only one of control.period_s or control.frequency_hz",
                                   out.name);

    if(hasPeriod)
      out.policyStepSize = control("period_s", 0.02);
    else if(hasFrequency)
    {
      const double frequency = control("frequency_hz", 50.0);
      if(frequency <= 0.0)
        mc_rtc::log::error_and_throw("[PolicyConfig:{}] control.frequency_hz must be positive", out.name);
      out.policyStepSize = 1.0 / frequency;
    }
    out.pdGainsRatio = control("pd_gains_ratio", 1.0);

    if(hasCmdVelSpeed)
    {
      VelocityCommand vel_cmd{control("vel_cmd_speed")("xy", 0.4), control("vel_cmd_speed")("yaw", 0.4)};
      out.velCmd = vel_cmd;
    }

    kp_map = control("kp", std::map<std::string, double>());
    kd_map = control("kd", std::map<std::string, double>());
    if(kp_map.size() < joint_size || kd_map.size() < mcRtcJoints.size())
      mc_rtc::log::error_and_throw("[PolicyConfig] policy.yaml: kp and kd must contain all joints");
  }
  else
    mc_rtc::log::error_and_throw("[PolicyConfig]: policy.yaml should contain a \"control\" entry");
  if(out.policyConfiguration.has("action"))
  {
    const mc_rtc::Configuration action = out.policyConfiguration("action");
    action("joints", out.actionJointGroup);
    out.controlledJointGroup = out.actionJointGroup;
    if(action.has("controlled_joints")) action("controlled_joints", out.controlledJointGroup);

    actionScale_map = action("scale", std::map<std::string, double>());
    if(actionScale_map.size() != 0)
      out.actionScale = std::vector<double>();
    else
      out.actionScale = std::vector<double>(joint_size, action("scale", 1.0));
    defaultPos_map = action("q0", std::map<std::string, double>());
    if(!defaultPos_map.empty() && defaultPos_map.size() != joint_size)
      mc_rtc::log::error_and_throw("[PolicyConfig] policy.yaml : default pos should contain all joints if specified");
    out.defaultPosition = std::vector<double>(defaultPos_map.size());
  }
  else
    mc_rtc::log::error_and_throw("[PolicyConfig]: policy.yaml should contain a \"action\" entry");

  for(size_t i = 0; i < joint_size; ++i)
  {
    const std::string & joint = mcRtcJoints[i];

    out.kp[i] = find_value(kp_map, joint, "kp", 0);
    out.kd[i] = find_value(kd_map, joint, "kd", 0);
    double scale = find_value(actionScale_map, joint, "action scale", -1);

    if(actionScale_map.size() > 0 && scale != -1) out.actionScale.push_back(scale);
    if(defaultPos_map.size() > 0) out.defaultPosition[i] = find_value(defaultPos_map, joint, "q0", 0);
  }

  if(out.controlledJointGroup.empty()) out.controlledJointGroup = out.actionJointGroup;

  out.validate();
  return out;
}

void PolicyConfig::validate() const
{
  if(name.empty()) mc_rtc::log::error_and_throw("[PolicyConfig] Policy name cannot be empty");

  if(folder.empty()) mc_rtc::log::error_and_throw("[PolicyConfig:{}] Policy folder cannot be empty", name);

  if(onnxPath.empty()) mc_rtc::log::error_and_throw("[PolicyConfig:{}] ONNX path cannot be empty", name);

  if(actionJointGroup.empty()) mc_rtc::log::error_and_throw("[PolicyConfig:{}] action.joints cannot be empty", name);

  if(policyStepSize <= 0.0) mc_rtc::log::error_and_throw("[PolicyConfig:{}] control.period_s must be positive", name);

  if(pdGainsRatio <= 0.0)
    mc_rtc::log::error_and_throw("[PolicyConfig:{}] control.pd_gains_ratio must be positive", name);
}

} // namespace rlqp
