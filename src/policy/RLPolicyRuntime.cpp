#include "policy/RLPolicyRuntime.h"

#include <mc_rtc/logging.h>

#include <algorithm>
#include <cmath>
#include <numeric>
#include <stdexcept>

namespace rlqp
{

RLPolicyRuntime::RLPolicyRuntime()
{
}

void RLPolicyRuntime::configure(const mc_rtc::Configuration & controllerConfig,
                                const std::string & policyFolder,
                                const std::string & onnxPath,
                                const std::shared_ptr<MCNNModel> & model,
                                mc_control::fsm::Controller & ctl,
                                const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  controllerConfig_ = controllerConfig;
  robotName_ = ctl.robot().name();
  controllerJointOrder_ = ctl.robot().refJointOrder();

  if(controllerConfig_.has("robot"))
  {
    baseBody_ = controllerConfig_("robot")("base_body", std::string("base_link"));
    observationSource_ = controllerConfig_("robot")("observation_source", std::string("realRobot"));
  }

  if(observationSource_ != "realRobot" && observationSource_ != "robot")
    mc_rtc::log::error_and_throw("[RLPolicyRuntime] Invalid robot.observation_source '{}'. Expected 'realRobot' or 'robot'.", observationSource_);

  const int nbActuatedJoints = static_cast<int>(controllerJointOrder_.size());

  q_rl_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  q_zero_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  currentActionScaled_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  actionScale_ = Eigen::VectorXd::Zero(nbActuatedJoints);

  kp_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  kd_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  kpBase_ = Eigen::VectorXd::Zero(nbActuatedJoints);
  kdBase_ = Eigen::VectorXd::Zero(nbActuatedJoints);

  observationRegistry_ = makeDefaultObservationRegistry();

  // mc_nn found the ONNX file and loaded the model; policy.yaml's `onnx` field is not used.
  policyConfig_ = PolicyConfig::load(policyFolder, controllerJointOrder_);
  policyConfig_.onnxPath = onnxPath;
  policy_ = model;
  loadPolicy(ctl, torqueTask);
}

void RLPolicyRuntime::reset(mc_control::fsm::Controller & ctl)
{
  phaseElapsedTime_ = 0.0;
  phaseNormalized_ = 0.0;
  resetObservationHistory(ctl);
  policyUpdateCount_ = 0;
}

void RLPolicyRuntime::advancePhase(double dt)
{
  phaseElapsedTime_ += dt;

  if(phasePeriod_ > 0.0)
    phaseNormalized_ = std::fmod(phaseElapsedTime_ / phasePeriod_, 1.0);
}

void RLPolicyRuntime::runPolicyStep(mc_control::fsm::Controller & ctl)
{
  if(!policyLoaded())
  {
    mc_rtc::log::error("[RLPolicyRuntime] Cannot run policy: no ONNX policy loaded");
    return;
  }

  currentObservation_ = computeObservation(ctl);

  if(currentObservation_.size() != observationSize())
    mc_rtc::log::error_and_throw("[RLPolicyRuntime] Observation size mismatch. ObservationManager produced {}, ONNX expects {}.",
      currentObservation_.size(), observationSize());

  try
  {
    currentAction_ = policy_->predict(currentObservation_);
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error("ONNX inference failed: {}", e.what());
    mc_rtc::log::error("Returning zero action as fallback");
    currentAction_ = Eigen::VectorXd::Zero(actionSize());
  }
  ++policyUpdateCount_;
  // mc_rtc::log::warning("TEST {}", currentAction_);

  if(currentAction_.size() != static_cast<int>(actionToControllerMap_.size()))
    mc_rtc::log::error_and_throw("[RLPolicyRuntime] Action size mismatch. ONNX produced {}, active action mapping expects {} joints.",
      currentAction_.size(), actionToControllerMap_.size());

  currentActionScaled_.setZero();
  q_rl_ = q_zero_;

  for(int actionIndex = 0; actionIndex < currentAction_.size(); ++actionIndex)
  {
    const int dofIndex = actionToControllerMap_[static_cast<size_t>(actionIndex)];

    currentActionScaled_(dofIndex) = actionScale_(dofIndex) * currentAction_(actionIndex);
    if(std::find(controlledActionControllerIndices_.begin(),
                 controlledActionControllerIndices_.end(),
                 dofIndex) != controlledActionControllerIndices_.end())
    {
      q_rl_(dofIndex) = currentActionScaled_(dofIndex) + q_zero_(dofIndex);
    }
  }
}

int RLPolicyRuntime::observationSize() const
{
  if(policyLoaded()) { return static_cast<int>(policy_->inputSize()); }
  return static_cast<int>(currentObservation_.size());
}

int RLPolicyRuntime::actionSize() const
{
  if(policyLoaded()) { return static_cast<int>(policy_->outputSize()); }
  return static_cast<int>(currentAction_.size());
}

void RLPolicyRuntime::setPDGainsRatio(double ratio,
                                      const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  pdGainsRatio_ = ratio;
  kp_ = pdGainsRatio_ * kpBase_;
  kd_ = std::sqrt(pdGainsRatio_) * kdBase_;

  if(torqueTask)
  {
    torqueTask->setStiffness(kp_);
    torqueTask->setDamping(kd_);
  }
}

void RLPolicyRuntime::loadPolicy(mc_control::fsm::Controller & ctl,
                                 const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  const PolicyConfig & policy = policyConfig_;

  mc_rtc::log::info("[RLPolicyRuntime] Loading policy '{}'", policy.name);

  configureControl(policy, ctl, torqueTask);

  const std::string conventionName =
    policy.observationsConfiguration("training_convention", std::string("mjlab"));
  activeConvention_ = ObservationConvention::fromConfig(controllerConfig_, conventionName);

  configureAction(policy, ctl);
  configureNetwork(policy);
  configureObservations(policy, ctl);

  phaseElapsedTime_ = 0.0;
  phaseNormalized_ = 0.0;
  resetObservationHistory(ctl);
  validateObservationAgainstNetwork();

  policyUpdateCount_ = 0;

  mc_rtc::log::success(
    "[RLPolicyRuntime] Policy '{}' loaded. Observation size: {}, action size: {}",
    policy.name, currentObservation_.size(), currentAction_.size());
}

void RLPolicyRuntime::configureControl(const PolicyConfig & policy,
                                       mc_control::fsm::Controller & ctl,
                                       const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask)
{
  useQP_ = policy.useQP;
  policyStepSize_ = policy.policyStepSize;
  pdGainsRatio_ = policy.pdGainsRatio;

  if(policyStepSize_ + 1e-8 < ctl.timeStep)
  {
    mc_rtc::log::warning(
        "[RLPolicyRuntime:{}] Policy period ({:.3f} ms) is shorter than the controller step ({:.3f} ms); "
        "the requested policy rate cannot be reached.",
        policy.name, 1000.0 * policyStepSize_, 1000.0 * ctl.timeStep);
  }

  phasePeriod_ = policy.observationsConfiguration("phase_period", 1.0);
  
  if(policy.policyConfiguration.has("control"))
  {
    const mc_rtc::Configuration control = policy.policyConfiguration("control");

    if(control.has("phase_period"))
      control("phase_period", phasePeriod_);
  }
  if(phasePeriod_ <= 0.0)
    mc_rtc::log::error_and_throw(
      "[RLPolicyRuntime:{}] phase_period must be positive, got {}",
      policy.name, phasePeriod_);

  kpBase_.setZero();
  kdBase_.setZero();

  kpBase_ = Eigen::Map<const Eigen::VectorXd>(policy.kp.data(), policy.kp.size());
  kdBase_ = Eigen::Map<const Eigen::VectorXd>(policy.kd.data(), policy.kp.size());

  kp_ = pdGainsRatio_ * kpBase_;
  kd_ = std::sqrt(pdGainsRatio_) * kdBase_;

  if(torqueTask)
  {
    torqueTask->setStiffness(kp_);
    torqueTask->setDamping(kd_);
  }
}

void RLPolicyRuntime::configureAction(const PolicyConfig & policy,
                                      mc_control::fsm::Controller & ctl)
{
  // Resolve action.joints to controller indices.
  // This is the ONNX action vector layout/size. Do not use controlled_joints here,
  // otherwise policies that output all joints but only apply legs will get a size mismatch.
  mc_rtc::Configuration actionSelector;
  
  if(!policy.actionJointGroup.empty())
    actionSelector.add("joints", policy.actionJointGroup);

  // Fallback: all controller joints in controller order
  std::vector<int> fullFallback(controllerJointOrder_.size());
  std::iota(fullFallback.begin(), fullFallback.end(), 0);

  actionToControllerMap_ = activeConvention_.resolveJointControllerIndices(
    actionSelector, controllerJointOrder_, fullFallback);

  // Resolve action.controlled_joints to controller indices.
  // This is the subset of the ONNX outputs that is actually applied to q_rl.
  mc_rtc::Configuration controlledSelector;
  controlledSelector.add("joints", policy.controlledJointGroup);
  controlledActionControllerIndices_ = activeConvention_.resolveJointControllerIndices(
    controlledSelector, controllerJointOrder_, actionToControllerMap_);

  // controlled_joints must be a subset of action.joints: every controlled joint must
  // correspond to one output in the ONNX action vector.
  for(const int controlledIndex : controlledActionControllerIndices_)
  {
    if(std::find(actionToControllerMap_.begin(), actionToControllerMap_.end(), controlledIndex)
       == actionToControllerMap_.end())
    {
      mc_rtc::log::error_and_throw(
        "[RLPolicyRuntime:{}] action.controlled_joints contains controller joint index {}, "
        "but this joint is not present in action.joints",
        policy.name,
        controlledIndex);
    }
  }

  mc_rtc::log::info("[RLPolicyRuntime:{}] action.joints='{}' -> controller indices {}",
                    policy.name, policy.actionJointGroup, actionToControllerMap_);
  mc_rtc::log::info("[RLPolicyRuntime:{}] action.controlled_joints='{}' -> controller indices {}",
                    policy.name, policy.controlledJointGroup, controlledActionControllerIndices_);

  q_zero_.setZero();
  q_rl_.setZero();
  actionScale_.setOnes();
  currentActionScaled_.setZero();

  actionScale_ = Eigen::Map<const Eigen::VectorXd>(policy.actionScale.data(), policy.actionScale.size());

  if (!policy.defaultPosition.empty())
  {
    q_zero_ = Eigen::Map<const Eigen::VectorXd>(policy.defaultPosition.data(), policy.defaultPosition.size());
  }
  else
  {
    size_t i = 0;
    std::shared_ptr<mc_tasks::PostureTask> FSMPostureTask = ctl.getPostureTask(ctl.robot().name());
    auto posture = FSMPostureTask->posture();
    for (const auto& j : ctl.robot().mb().joints()) {
      const std::string& joint_name = j.name();
      if (j.type() == rbd::Joint::Type::Rev) {
        if (const auto& t = posture[ctl.robot().jointIndexByName(joint_name)]; !t.empty()) {
          q_zero_(i) =t[0];
          i++;
        }
      }
    }
  }

  for(size_t actionIndex = 0; actionIndex < actionToControllerMap_.size(); ++actionIndex)
  {
    const int dofIndex = actionToControllerMap_[actionIndex];
    const std::string & joint = controllerJointOrder_[static_cast<size_t>(dofIndex)];

    if(!ctl.robot().hasJoint(joint))
    {
      mc_rtc::log::error_and_throw(
        "[RLPolicyRuntime:{}] Resolved action joint '{}' does not exist on robot",
        policy.name,
        joint);
    }
  }

  q_rl_ = q_zero_;
}

void RLPolicyRuntime::configureNetwork(const PolicyConfig & policy)
{
  // The ONNX session itself is created and owned by mc_nn (MCNNModel).
  if(!policyLoaded())
    mc_rtc::log::error_and_throw("[RLPolicyRuntime:{}] RL policy creation failed for '{}'",
      policy.name, policy.onnxPath);

  currentAction_ = Eigen::VectorXd::Zero(actionSize());

  if(static_cast<int>(actionToControllerMap_.size()) != actionSize())
    mc_rtc::log::error_and_throw("[RLPolicyRuntime:{}] Resolved action joint count ({}) does not match ONNX action size ({})",
      policy.name, actionToControllerMap_.size(), actionSize());
}

void RLPolicyRuntime::configureObservations(const PolicyConfig & policy,
                                            mc_control::fsm::Controller & ctl)
{
  observationManager_.load(policy.observationsConfiguration, controllerConfig_, observationRegistry_);
  observationManager_.configure(makeObservationContext(ctl));
  currentObservation_ = Eigen::VectorXd::Zero(observationSize());
}

void RLPolicyRuntime::resetObservationHistory(mc_control::fsm::Controller & ctl)
{
  if(!policyLoaded())
  {
    mc_rtc::log::error("[RLPolicyRuntime] Cannot reset observation history: no policy loaded");
    return;
  }

  ObservationContext context = makeObservationContext(ctl);

  observationManager_.updateHistory(context);
  currentObservation_ = observationManager_.compute(context);
}

Eigen::VectorXd RLPolicyRuntime::computeObservation(mc_control::fsm::Controller & ctl)
{
  ObservationContext context = makeObservationContext(ctl);
  return observationManager_.compute(context);
}

void RLPolicyRuntime::validateObservationAgainstNetwork() const
{
  if(!policyLoaded())
    mc_rtc::log::error_and_throw("[RLPolicyRuntime] Cannot validate observation: no policy loaded");

  if(currentObservation_.size() != observationSize())
    mc_rtc::log::error_and_throw("[RLPolicyRuntime] ObservationManager dimension ({}) does not match ONNX input size ({})",
      currentObservation_.size(), observationSize());
}

ObservationContext RLPolicyRuntime::makeObservationContext(mc_control::fsm::Controller & ctl)
{
  return ObservationContext{
    selectedObservationRobot(ctl),
    baseBody_,
    controllerJointOrder_,
    actionToControllerMap_,
    q_zero_,
    currentAction_,
    command_,
    phaseNormalized_,
    activeConvention_};
}

mc_rbdyn::Robot & RLPolicyRuntime::selectedObservationRobot(mc_control::fsm::Controller & ctl)
{
  if(observationSource_ == "robot")
    return ctl.robot();
  return ctl.realRobot(robotName_);
}


void RLPolicyRuntime::addLogObs(mc_control::fsm::Controller & ctl, const std::string & prefix, const void * source)
{
  for(const auto & entry : observationManager_.entries())
  {
    const std::string baseName = prefix + "Observations_" + entry.observation->name();
    const size_t size = entry.historyBuffer.size();
    if(size == 1)
    {
      ctl.logger().addLogEntry(baseName, source, [entry]() { return entry.historyBuffer[0]; });
    }
    else if(size > 1)
    {
      for(size_t i = 0; i < size; ++i)
      {
        const size_t bufferIndex = observationManager_.newest_first() ? i : size - 1 - i;
        const std::string suffix = i == 0 ? "_t" : "_t-" + std::to_string(i);
        ctl.logger().addLogEntry(baseName + suffix, source,
                                 [entry, bufferIndex]() { return entry.historyBuffer[bufferIndex]; });
      }
    }
  }
}

} // namespace rlqp
