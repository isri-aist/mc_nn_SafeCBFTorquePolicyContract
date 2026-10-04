#pragma once

#include "policy/RLPolicyRuntime.h"

#include "mc_nn/MCNNContract.h"
#include "mc_nn/MCNNModel.h"

#include <Eigen/Core>

#include <mc_control/fsm/Controller.h>
#include <mc_rtc/Configuration.h>
#include <mc_solver/DynamicsConstraint.h>
#include <mc_tasks/PostureTask.h>
#include <mc_tasks/TorqueJointTask.h>

#include <memory>
#include <string>
#include <vector>

/**
 * @brief mc_nn contract deploying one reinforcement learning policy on real robots
 * through a torque-level CBF-QP (formerly the NewRLQPController template).
 *
 * This contract bridges a trained RL policy (exported as ONNX) with mc_rtc's
 * whole-body QP framework augmented with Control Barrier Functions (CBFs).
 *
 * ## Control Pipeline
 *
 * At each policy step (typically 20ms):
 *   1. Build the observation vector from robot state (joint positions, velocities,
 *      IMU data, contact forces, velocity commands, etc.)
 *   2. Run inference: action = policy(observation)
 *   3. Compute position target: q* = action * action_scale + q_zero
 *   4. Compute desired torque via PD control: τ = Kp*(q* - q) - Kd*q̇
 *   5. Either:
 *      - (useQP=true)  Feed τ as input to TorqueJointTask inside the CBF-QP solver,
 *        which enforces joint limits, velocity limits, and self-collision constraints.
 *      - (useQP=false) Apply τ directly to the robot joints, bypassing the QP.
 *
 * ## Torque Equation
 *
 *   τ_i = clip(Kp_i * (q*_i - q_i) - Kd_i * q̇_i,  ±effort_limit_i)
 *
 * where:
 *   - q*_i  = action_i * action_scale_i + q_zero_i   (position target)
 *   - Kp_i  = pd_gains_ratio * kp_base_i
 *   - Kd_i  = sqrt(pd_gains_ratio) * kd_base_i
 *
 * ## Configuration
 *
 * mc_nn finds the policy's ONNX file (YAML `onnx` patterns in `models_dirs`) and
 * owns inference, scheduling and policy switching. This contract reads:
 * RunNN policies entry (contract-specific fields):
 *  - robot.base_body / robot.observation_source
 *  - conventions_file            conventions.yaml path (default: <policy folder>/../conventions.yaml)
 * {policy folder} (the directory containing the .onnx file):
 * policy.yaml :
 *  - action_scale:      Per-joint scale applied to raw policy output (map: joint -> scale)
 *  - q0:                Reference joint positions (default pose), in radians
 *  - kp / kd:           PD gains per joint
 *  - use_QP:            Whether to route torques through the CBF-QP (true) or apply directly (false)
 *  - pd_gains_ratio:    Runtime gain scaling factor (1.0 = nominal gains)
 *  - action_joints:     Joints on which to apply the action. Can be a pre-registered name such as "legs"
 *  - period_s:          Policy inference period in seconds (mc_nn rate when YAML policy_hz is 0)
 *  - frequency_hz:      Alternative to period_s; policy inference frequency in hertz
 *  observations.yaml :
 *  - training_convention     Convention to load default values and aliases from
 *  - list of observations and their parameters
 * conventions.yaml : stores all known conventions (mjlab, isaaclab)
 *  - joint groups in training order
 *  - type aliases to correspond precisely to the name in the training environment.
 *  - observations defaults
 *
 * Contact forces must be log-compressed before insertion:
 *   f_obs = sign(f) * log(1 + |f|)
 *
 * ## Host controller requirements
 *
 * - Torque control and the forked mc_rtc/TVM providing mc_tasks::TorqueJointTask (see README).
 * - The host must call the "MCNN::AfterSolve" datastore entry after its QP solve
 *   (useQP=false writes torques after the solve), and should run with
 *   FeedbackType: ClosedLoopIntegrateReal. MCNNController supports both; see mc_nn/MCNNHost.h for other hosts.
 * - The host FSM must not declare its own `dynamics` constraint: this contract
 *   installs the CBF dynamics constraint while it runs.
 *
 * The contract is exclusive by default: it drives every actuated joint in torque.
 *
 * @see RLPolicyRuntime.h for the observation/action pipeline.
 */
struct mc_nn_SafeCBFTorquePolicyContract : public MCNNContract
{
  // ---- mc_nn contract hooks
  void configure(const mc_rtc::Configuration & config) override;
  bool load() override;
  bool loaded() const override { return model_ != nullptr; }
  /** control.period_s / control.frequency_hz from policy.yaml. */
  double defaultRateHz() const override { return 1.0 / policyStepSize_; }
  /** Drives every actuated joint in torque: pauses other policies unless YAML `exclusive: false`. */
  bool defaultExclusive() const override { return true; }
  /** useQP=false applies torques after the QP solve. */
  bool requiresAfterSolve() const override { return true; }

  /** Install constraints and the torque task, load the policy configuration (former controller constructor/reset). */
  void start(mc_control::fsm::Controller & ctl) override;
  /** Command inputs, limit checks, phase and PD target, every controller tick (former controller run()). */
  void update(mc_control::fsm::Controller & ctl, double dt) override;
  /** One inference step at the policy rate. */
  void step(mc_control::fsm::Controller & ctl) override;
  /** Apply RL torques directly when the QP is bypassed (useQP=false). */
  void afterSolve(mc_control::fsm::Controller & ctl) override;
  /** Remove the torque task and restore the posture task, dynamics constraint and ControlMode. */
  void teardown(mc_control::fsm::Controller & ctl) override;

  /** Register GUI elements specific to this contract (QP toggle, gains, commands...). */
  void addGui(mc_control::fsm::Controller & ctl, const std::vector<std::string> & category) override;
  /** Register data entries visible in mc_log_ui. */
  void addLog(mc_control::fsm::Controller & ctl, const std::string & prefix) override;

  size_t observationSize() const override { return model_ ? model_->inputSize() : 0; }
  size_t actionSize() const override { return model_ ? model_->outputSize() : 0; }
  Eigen::VectorXd observation() const override { return rlRuntime_.currentObservation(); }
  Eigen::VectorXd action() const override { return rlRuntime_.currentAction(); }

  void RLuseJoyStickInputs(mc_control::fsm::Controller & ctl);
  void RLuseKeyboardInputs();

  /** @brief Enable or disable the CBF-QP layer at runtime. */
  void activateQPControl(bool activate);

  rlqp::RLPolicyRuntime & rlRuntime();
  const rlqp::RLPolicyRuntime & rlRuntime() const;

  void setMaxVelCmd(double new_max_vel_cmd)
  {
    maxVelCmd = new_max_vel_cmd;
  };
  void setMaxYawCmd(double new_max_yaw_cmd)
  {
    maxYawCmd = new_max_yaw_cmd;
  };

  /** @brief Torque-space whole-body task fed into the CBF-QP solver. */
  std::shared_ptr<mc_tasks::TorqueJointTask> torqueJointTask;

  /** @brief Total number of actuated joints (from robot().refJointOrder()). */
  int nbActuatedJoints = 0;

  /**
   * @brief Joint names in mc_rtc's reference order (robot().refJointOrder()).
   *
   * This order is used as the canonical ordering for all Eigen vectors
   * in this contract (kp_, kd_, q_zero, q_rl, etc.).
   */
  std::vector<std::string> jointNames;

private:
  /** @brief Configuration given to RLPolicyRuntime: robot block and conventions. */
  mc_rtc::Configuration config_;

  /** @brief Load robot parameters and switch the host to torque control. */
  void initializeRobotBasics(mc_control::fsm::Controller & ctl);

  /**
   * @brief Apply RL torques directly, bypassing the QP (useQP=false mode).
   *
   * Computes τ = Kp*(q_rl - q) - Kd*q̇ and writes it to robot().mbc().jointTorque.
   * @return true if bypass was applied, false if QP should run instead.
   */
  bool byPassQPControl(mc_control::fsm::Controller & ctl);
  /** @brief Log warnings when joint position/velocity/torque limits are exceeded. */
  void computeLimits(mc_control::fsm::Controller & ctl);

private:
  bool printLimits_ = true;
  bool toggleJoystick = true;
  bool toggleKeyboard = false;
  bool controllerAvailable = false;

  std::string robotName_;

  // --- CBF-QP constraint parameters ---
  double velPercent_ = 0.9; // Percentage of the max velocity taking account in the joint velocity constraint.
  double dsPercent_ = 0.01; // Percentage of the max joint range taking account in the joint position limit constraint.
  double diPercent_ = 0.1; // Doesn't matter since di > ds. This variable is not used in the constraint dynamics.

  // --- CBF Gains ---
  // More details are explained in the paper cf. Readme.md.
  // TODO(robot) : These vealues must be tuned depending on the robot.
  double zeta_jointLimit_ = 1.2;
  double lambda_jointLimit_ = 200.0; // Same gain for joint position limits and velocity limits.
  double zeta_selfCollision_ = 1.2;
  double lambda_selfCollision_ = 200.0;

  rlqp::RLPolicyRuntime rlRuntime_;
  /** @brief ONNX model loaded by mc_nn's core (MCNNModel). */
  std::shared_ptr<MCNNModel> model_;
  /** @brief Policy period from policy.yaml, read in configure() so mc_nn knows the default rate. */
  double policyStepSize_ = 0.02;

  // --- State restored by teardown() ---
  mc_rtc::unique_ptr<mc_solver::DynamicsConstraint> hostDynamicsConstraint_;
  bool dynamicsInstalled_ = false;
  std::shared_ptr<mc_tasks::PostureTask> hostPostureTask_;
  bool hostPostureTaskInSolver_ = false;
  bool controlModeSet_ = false;
  bool hadControlMode_ = false;
  std::string hostControlMode_;

  // --- Velocity Cmd Params ---
  std::vector<bool> DirectionButtons = std::vector<bool>(4, false); // Up, Down, Left, Right
  double joystickDeadZone = 0.02; // Dead zone for joystick inputs
  Eigen::Vector2d leftStick = Eigen::Vector2d(0.5, 0.5); // x (UP), y (LEFT)
  Eigen::Vector2d rightStick = Eigen::Vector2d(0.5, 0.5); // x (UP), y (LEFT)

  double maxVelCmd;
  double maxYawCmd;
};
