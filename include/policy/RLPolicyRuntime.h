#pragma once

#include "observation/Observation.h"
#include "observation/ObservationManager.h"
#include "policy/PolicyConfig.h"

#include "mc_nn/MCNNModel.h"

#include <Eigen/Core>

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/Configuration.h>
#include <mc_rbdyn/Robot.h>
#include <mc_tasks/TorqueJointTask.h>

#include <memory>
#include <string>
#include <vector>

namespace rlqp
{

  /**
 * @brief Runtime owner of the active RL policy session.
 *
 * Responsibilities:
 *
 * - Load the policy configuration (policy.yaml, observations.yaml).
 * - Run the ONNX model provided by mc_nn (MCNNModel).
 * - Own the ObservationManager.
 * - Build observation vectors.
 * - Execute policy inference.
 * - Convert policy outputs into q_rl targets.
 * - Manage policy gains and action scaling.
 *
 * mc_nn_SafeCBFTorquePolicyContract remains responsible for the mc_rtc lifecycle, and mc_nn
 * for policy discovery, switching and scheduling.
 * RLPolicyRuntime owns everything that is policy-dependent.
 */
class RLPolicyRuntime
{
public:
  RLPolicyRuntime();

  /**
   * Load the policy in `policyFolder` (policy.yaml, observations.yaml) with the
   * ONNX model loaded by mc_nn from `onnxPath`.
   */
  void configure(const mc_rtc::Configuration & controllerConfig,
                 const std::string & policyFolder,
                 const std::string & onnxPath,
                 const std::shared_ptr<MCNNModel> & model,
                 mc_control::fsm::Controller & ctl,
                 const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  void reset(mc_control::fsm::Controller & ctl);
  /** Advance the phase clock; called every controller timestep. */
  void advancePhase(double dt);
  /** One inference step (observation, ONNX, q_rl); called by mc_nn at the policy rate. */
  void runPolicyStep(mc_control::fsm::Controller & ctl);

  bool policyLoaded() const { return policy_ != nullptr; }

  const PolicyConfig & currentPolicy() const { return policyConfig_; }
  const std::string & currentPolicyName() const { return policyConfig_.name; }
  const std::string & currentPolicyFolder() const { return policyConfig_.folder; }
  const std::string & conventionName() const { return observationManager_.conventionName(); }

  int observationSize() const;
  int actionSize() const;

  double policyStepSize() const { return policyStepSize_; }
  double policyRate() const { return 1.0 / policyStepSize_; }
  size_t policyUpdateCount() const { return policyUpdateCount_; }
  const std::string & observationSource() const { return observationSource_; }
  const std::string & baseBody() const { return baseBody_; }
  size_t controlledActionSize() const { return controlledActionControllerIndices_.size(); }
  double phase() const { return phaseNormalized_; }

  bool useQP() const { return useQP_; }
  void setUseQP(bool useQP) { useQP_ = useQP; }

  const Eigen::VectorXd & q_rl() const { return q_rl_; }
  const Eigen::VectorXd & q_zero() const { return q_zero_; }
  const Eigen::VectorXd & currentObservation() const { return currentObservation_; }
  const Eigen::VectorXd & currentAction() const { return currentAction_; }
  const Eigen::VectorXd & currentActionScaled() const { return currentActionScaled_; }
  const Eigen::VectorXd & actionScale() const { return actionScale_; }

  const Eigen::VectorXd & kp() const { return kp_; }
  const Eigen::VectorXd & kd() const { return kd_; }
  const Eigen::VectorXd & kpBase() const { return kpBase_; }
  const Eigen::VectorXd & kdBase() const { return kdBase_; }

  double pdGainsRatio() const { return pdGainsRatio_; }

  void setPDGainsRatio(double ratio,
                       const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  /** @brief Add one logger entry per configured observation/history element, named prefix + "Observations_" + name. */
  void addLogObs(mc_control::fsm::Controller & ctl, const std::string & prefix, const void * source);

  Eigen::Vector3d & command() { return command_; }
  const Eigen::Vector3d & command() const { return command_; }
  void setCommand(Eigen::Vector3d new_cmd)
  {
    command_ = new_cmd;
  }

private:
  void loadPolicy(mc_control::fsm::Controller & ctl,
                  const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  void configureControl(const PolicyConfig & policy,
                        mc_control::fsm::Controller & ctl,
                        const std::shared_ptr<mc_tasks::TorqueJointTask> & torqueTask);

  void configureAction(const PolicyConfig & policy, mc_control::fsm::Controller & ctl);
  void configureNetwork(const PolicyConfig & policy);
  void configureObservations(const PolicyConfig & policy, mc_control::fsm::Controller & ctl);

  void resetObservationHistory(mc_control::fsm::Controller & ctl);
  Eigen::VectorXd computeObservation(mc_control::fsm::Controller & ctl);

  void validateObservationAgainstNetwork() const;
  ObservationContext makeObservationContext(mc_control::fsm::Controller & ctl);
  mc_rbdyn::Robot & selectedObservationRobot(mc_control::fsm::Controller & ctl);

private:
  mc_rtc::Configuration controllerConfig_;

  /** Configuration of this policy. */
  PolicyConfig policyConfig_;
  /** Observation type factory registry. */
  ObservationRegistry observationRegistry_;
  /** Active observation pipeline. */
  ObservationManager observationManager_;
  /** Active training-environment convention. */
  ObservationConvention activeConvention_;
  /** ONNX model loaded by mc_nn. */
  std::shared_ptr<MCNNModel> policy_;

  std::string robotName_;
  std::string baseBody_ = "base_link";
  std::string observationSource_ = "realRobot";

  std::vector<std::string> controllerJointOrder_;
  /** @brief Maps active policy action index to controllerJointOrder_ index. policyJointControllerIndices in ObservationContext. */
  std::vector<int> actionToControllerMap_;

  /** @brief Controller-order joint indices that are actually allowed to receive the policy action. */
  std::vector<int> controlledActionControllerIndices_;

  Eigen::VectorXd q_rl_;
  Eigen::VectorXd q_zero_;
  Eigen::VectorXd currentObservation_;
  Eigen::VectorXd currentAction_;
  Eigen::VectorXd currentActionScaled_;
  Eigen::VectorXd actionScale_;

  Eigen::VectorXd kp_;
  Eigen::VectorXd kd_;
  Eigen::VectorXd kpBase_;
  Eigen::VectorXd kdBase_;

  Eigen::Vector3d command_ = Eigen::Vector3d::Zero();

  bool useQP_ = true;
  double policyStepSize_ = 0.02;
  size_t policyUpdateCount_ = 0;

  double phasePeriod_ = 1.0;
  double phaseElapsedTime_ = 0.0;
  double phaseNormalized_ = 0.0;

  double pdGainsRatio_ = 1.0;
};

} // namespace rlqp
