#include "mc_nn_SafeCBFTorquePolicyContract.h"

#include "mc_nn/MCNNHost.h"
#include "mc_nn/MCNNRegistry.h"

#include <RBDyn/MultiBodyConfig.h>

#include <mc_rtc/gui.h>
#include <mc_rtc/logging.h>

#include <fcntl.h>
#ifdef MC_NN_HAS_JOYSTICK_PLUGIN
#include <mc_joystick_plugin/joystick_inputs.h>
#endif
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <filesystem>

void mc_nn_SafeCBFTorquePolicyContract::configure(const mc_rtc::Configuration & config)
{
  // RLPolicyRuntime reads the robot block and the conventions from this configuration.
  if(config.has("robot")) { config_.add("robot", config("robot")); }

  std::string conventionsFile = config("conventions_file", std::string{});
  if(conventionsFile.empty())
  {
    conventionsFile = (std::filesystem::path(info().folder).parent_path() / "conventions.yaml").string();
  }
  mc_rtc::Configuration conventions(conventionsFile);
  if(!conventions.has("conventions"))
  {
    mc_rtc::log::error_and_throw("[mc_nn_SafeCBFTorquePolicyContract:{}] '{}' has no 'conventions' entry", info().id, conventionsFile);
  }
  config_.add("conventions", conventions("conventions"));

  // Default mc_nn rate: same rules as PolicyConfig (control.period_s or control.frequency_hz, 0.02 s otherwise).
  const mc_rtc::Configuration policy((std::filesystem::path(info().folder) / "policy.yaml").string());
  if(policy.has("control"))
  {
    const mc_rtc::Configuration control = policy("control");
    if(control.has("period_s")) { policyStepSize_ = control("period_s"); }
    else if(control.has("frequency_hz")) { policyStepSize_ = 1.0 / static_cast<double>(control("frequency_hz")); }
  }
  if(policyStepSize_ <= 0.0)
  {
    mc_rtc::log::error_and_throw("[mc_nn_SafeCBFTorquePolicyContract:{}] control.period_s must be positive", info().id);
  }
}

bool mc_nn_SafeCBFTorquePolicyContract::load()
{
  try
  {
    model_ = std::make_shared<MCNNModel>(info().modelPath, info().device, info().id);
    return true;
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error("[mc_nn_SafeCBFTorquePolicyContract:{}] Failed to load ONNX policy '{}': {}", info().id, info().modelPath, e.what());
    model_.reset();
    return false;
  }
}

void mc_nn_SafeCBFTorquePolicyContract::start(mc_control::fsm::Controller & ctl)
{
  if(mc_nn::host::feedbackType(ctl) != "ClosedLoopIntegrateReal")
  {
    mc_rtc::log::warning("[mc_nn_SafeCBFTorquePolicyContract:{}] Host controller FeedbackType is '{}', this contract was designed for "
                         "ClosedLoopIntegrateReal",
                         info().id, mc_nn::host::feedbackType(ctl));
  }

  //Initialize Constraints
  // TODO(robot): review which constraints are valid for the selected robot.
  // These defaults provide joint-limit and self-collision protection
  ctl.selfCollisionConstraint->setCollisionsDampers(ctl.solver(), {zeta_selfCollision_, lambda_selfCollision_});
  ctl.solver().removeConstraintSet(ctl.dynamicsConstraint);
  hostDynamicsConstraint_ = std::move(ctl.dynamicsConstraint);
  ctl.dynamicsConstraint = mc_rtc::unique_ptr<mc_solver::DynamicsConstraint>(
    new mc_solver::DynamicsConstraint(ctl.robots(), 0, {diPercent_, dsPercent_, 0.0, zeta_jointLimit_, lambda_jointLimit_}, velPercent_, true));
  ctl.solver().addConstraintSet(ctl.dynamicsConstraint);
  dynamicsInstalled_ = true;

  // Remove the default posture task created by the FSM
  hostPostureTask_ = ctl.getPostureTask(ctl.robot().name());
  const auto & tasks = ctl.solver().tasks();
  hostPostureTaskInSolver_ = hostPostureTask_ && std::find(tasks.begin(), tasks.end(), hostPostureTask_.get()) != tasks.end();
  if(hostPostureTaskInSolver_) { ctl.solver().removeTask(hostPostureTask_); }
  // Initialize Task
  torqueJointTask = std::make_shared<mc_tasks::TorqueJointTask>(
      ctl.solver(), ctl.robot().robotIndex(), 100.0, 1); // TODO(robot): tune stiffness/weight
  ctl.solver().addTask(torqueJointTask);
  initializeRobotBasics(ctl);

  rlRuntime_.configure(config_, info().folder, info().modelPath, model_, ctl, torqueJointTask);
  setMaxVelCmd(rlRuntime_.currentPolicy().velCmd.xy);
  setMaxYawCmd(rlRuntime_.currentPolicy().velCmd.yaw);
  rlRuntime_.reset(ctl);

  mc_rtc::log::success("[mc_nn_SafeCBFTorquePolicyContract:{}] started", info().id);
}

void mc_nn_SafeCBFTorquePolicyContract::update(mc_control::fsm::Controller & ctl, double dt)
{
  // Use joystick plugin if present else jeyboard inputs
  controllerAvailable = ctl.datastore().has("Joystick::connected") && ctl.datastore().get<bool>("Joystick::connected");
  if(controllerAvailable && toggleJoystick)
    RLuseJoyStickInputs(ctl);
  else if(toggleKeyboard)
    RLuseKeyboardInputs();

  if(printLimits_) computeLimits(ctl);

  rlRuntime_.advancePhase(dt);
  // Between inference steps, q_rl is held constant and the PD torque uses fresh joint state.
  torqueJointTask->setPosTarget(rlRuntime_.q_rl());
}

void mc_nn_SafeCBFTorquePolicyContract::step(mc_control::fsm::Controller & ctl)
{
  try
  {
    rlRuntime_.runPolicyStep(ctl);
  }
  catch(const std::exception & e)
  {
    mc_rtc::log::error("[mc_nn_SafeCBFTorquePolicyContract:{}] Error during RL policy step: {}", info().id, e.what());
  }
  torqueJointTask->setPosTarget(rlRuntime_.q_rl());
}

void mc_nn_SafeCBFTorquePolicyContract::afterSolve(mc_control::fsm::Controller & ctl)
{
  byPassQPControl(ctl); // Run RL without taking the QP into account
}

void mc_nn_SafeCBFTorquePolicyContract::teardown(mc_control::fsm::Controller & ctl)
{
  if(torqueJointTask)
  {
    ctl.solver().removeTask(torqueJointTask);
    torqueJointTask.reset();
  }

  // Only undo what start() did: RunNN also calls teardown() after a failed start().
  if(dynamicsInstalled_)
  {
    ctl.solver().removeConstraintSet(ctl.dynamicsConstraint);
    ctl.dynamicsConstraint = std::move(hostDynamicsConstraint_);
    dynamicsInstalled_ = false;
  }
  // setCollisionsDampers has no getter: the self-collision dampers keep this contract's values.

  if(hostPostureTaskInSolver_)
  {
    hostPostureTask_->reset(); // Hold the current posture instead of jumping back to the old target
    ctl.solver().addTask(hostPostureTask_);
    hostPostureTaskInSolver_ = false;
  }
  hostPostureTask_.reset();

  if(controlModeSet_)
  {
    if(hadControlMode_) { ctl.datastore().assign<std::string>("ControlMode", hostControlMode_); }
    else if(ctl.datastore().has("ControlMode")) { ctl.datastore().remove("ControlMode"); }
    controlModeSet_ = false;
  }
}

void mc_nn_SafeCBFTorquePolicyContract::RLuseJoyStickInputs(mc_control::fsm::Controller & ctl)
{
#ifdef MC_NN_HAS_JOYSTICK_PLUGIN
  // Get joystick functions
  auto & stickFunc = ctl.datastore().get<std::function<Eigen::Vector2d(joystickAnalogicInputs)>>("Joystick::Stick");

  // Read sticks values
  leftStick = stickFunc(joystickAnalogicInputs::L_STICK);
  // Apply dead zone
  double vel_x = 0.0;
  if(std::abs(leftStick(0) - 0.5) > joystickDeadZone)
  {
    vel_x = (leftStick(0) - 0.5) * 2.0 * maxVelCmd;
  }
  double vel_y = 0.0;
  if(std::abs(leftStick(1) - 0.5) > joystickDeadZone)
  {
    vel_y = (leftStick(1) - 0.5) * 2.0 * maxVelCmd;
  }

  rightStick = stickFunc(joystickAnalogicInputs::R_STICK);
  double yaw_cmd = 0.0;
  if(std::abs(rightStick(1) - 0.5) > joystickDeadZone)
  {
    yaw_cmd = (rightStick(1) - 0.5) * 2.0 * maxYawCmd;
  }

  // Read D-pad buttons
  DirectionButtons = {ctl.datastore().get<bool>("Joystick::UpPad"), ctl.datastore().get<bool>("Joystick::DownPad"),
                      ctl.datastore().get<bool>("Joystick::LeftPad"), ctl.datastore().get<bool>("Joystick::RightPad")};

  for(size_t i = 0; i < DirectionButtons.size(); ++i)
  {
    if(DirectionButtons[i])
    {
      switch(i)
      {
        case 0: // Up
          vel_x += 1.0 * maxVelCmd;
          break;
        case 1: // Down
          vel_x -= 1.0 * maxVelCmd;
          break;
        case 2: // Left
          vel_y += 1.0 * maxVelCmd;
          break;
        case 3: // Right
          vel_y -= 1.0 * maxVelCmd;
          break;
        default:
          break;
      }
    }
  }
  rlRuntime_.setCommand({vel_x, vel_y, yaw_cmd});
#else
  (void)ctl; // Built without mc_joystick_plugin: joystick commands are unavailable
#endif
}

void mc_nn_SafeCBFTorquePolicyContract::RLuseKeyboardInputs()
{
  struct Ctx
  {
    bool ready = false;
    termios old{};
    bool seen[4] = {};
    std::chrono::steady_clock::time_point ts[4];
    std::array<char, 64> buf{};
    size_t sz = 0;
  };
  static Ctx k;

  if(!k.ready)
  {
    if(::isatty(STDIN_FILENO) != 1) return;
    k.ready = true;
    ::tcgetattr(STDIN_FILENO, &k.old);
    termios raw = k.old;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = raw.c_cc[VTIME] = 0;
    ::tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    ::fcntl(STDIN_FILENO, F_SETFL, ::fcntl(STDIN_FILENO, F_GETFL, 0) | O_NONBLOCK);
  }

  char tmp[32];
  ssize_t n = ::read(STDIN_FILENO, tmp, sizeof(tmp));
  if(n > 0 && k.sz + n < 64) std::copy(tmp, tmp + n, k.buf.begin() + k.sz), k.sz += n;

  auto now = std::chrono::steady_clock::now();
  for(size_t i = 0; i + 2 < k.sz; ++i)
    if(k.buf[i] == 27 && k.buf[i + 1] == '[')
    {
      int idx = k.buf[i + 2] == 'A'   ? 0
                : k.buf[i + 2] == 'B' ? 1
                : k.buf[i + 2] == 'D' ? 2
                : k.buf[i + 2] == 'C' ? 3
                                      : -1;
      if(idx >= 0) k.seen[idx] = true, k.ts[idx] = now;
      i += 2;
    }
  std::copy(k.buf.begin() + (k.sz > 2 ? k.sz - 2 : 0), k.buf.begin() + k.sz, k.buf.begin());
  k.sz = k.sz > 2 ? 2 : 0;

  const auto active = [&](int i)
  { return k.seen[i] && std::chrono::duration_cast<std::chrono::milliseconds>(now - k.ts[i]).count() < 500; };
  rlRuntime_.setCommand({(active(0) ? maxVelCmd : 0.0) - (active(1) ? maxVelCmd : 0.0),
                         (active(2) ? maxVelCmd : 0.0) - (active(3) ? maxVelCmd : 0.0), rlRuntime_.command()(2)});
}

void mc_nn_SafeCBFTorquePolicyContract::activateQPControl(bool activate)
{
  rlRuntime_.setUseQP(activate);
}

rlqp::RLPolicyRuntime & mc_nn_SafeCBFTorquePolicyContract::rlRuntime() { return rlRuntime_; }

const rlqp::RLPolicyRuntime & mc_nn_SafeCBFTorquePolicyContract::rlRuntime() const { return rlRuntime_; }

void mc_nn_SafeCBFTorquePolicyContract::initializeRobotBasics(mc_control::fsm::Controller & ctl)
{
  mc_rtc::log::info("[mc_nn_SafeCBFTorquePolicyContract:{}] Using torque control mode", info().id);
  hadControlMode_ = ctl.datastore().has("ControlMode");
  if(!hadControlMode_)
  {
    ctl.datastore().make<std::string>("ControlMode", "Torque");
  }
  else
  {
    hostControlMode_ = ctl.datastore().get<std::string>("ControlMode");
    ctl.datastore().assign<std::string>("ControlMode", "Torque");
  }
  controlModeSet_ = true;

  robotName_ = ctl.robot().name();
  jointNames = ctl.robot().refJointOrder();
  nbActuatedJoints = static_cast<int>(jointNames.size());
}

bool mc_nn_SafeCBFTorquePolicyContract::byPassQPControl(mc_control::fsm::Controller & ctl)
{
  if(rlRuntime_.useQP()) return false; // QP is not bypassed, do nothing

  auto & robot = ctl.robot();
  robot.forwardKinematics();
  robot.forwardVelocity();
  robot.forwardAcceleration();

  Eigen::VectorXd tau_rl = Eigen::VectorXd::Zero(nbActuatedJoints);
  const std::vector<std::vector<double> > & q_mbc = robot.mbc().q;
  const std::vector<std::vector<double> > & q_dot_mbc = robot.mbc().alpha;

  int i = 0;
  for(const auto &joint_name : jointNames)
  {
    const double q = q_mbc[robot.jointIndexByName(joint_name)][0];
    const double q_dot = q_dot_mbc[robot.jointIndexByName(joint_name)][0];
    tau_rl(i) = rlRuntime_.kp()(i) * (rlRuntime_.q_rl()(i) - q) - rlRuntime_.kd()(i) * q_dot;
    robot.mbc().jointTorque[robot.jointIndexByName(joint_name)][0] = tau_rl(i);
    i++;
  }

  return true;
}

void mc_nn_SafeCBFTorquePolicyContract::addLog(mc_control::fsm::Controller & ctl, const std::string & prefix)
{
  auto & logger = ctl.logger();
  // Robot State variables
  logger.addLogEntry(prefix + "kp_base", this, [this]() { return rlRuntime_.kpBase(); });
  logger.addLogEntry(prefix + "kd_base", this, [this]() { return rlRuntime_.kdBase(); });
  logger.addLogEntry(prefix + "kp_current", this, [this]() { return rlRuntime_.kp(); });
  logger.addLogEntry(prefix + "kd_current", this, [this]() { return rlRuntime_.kd(); });
  logger.addLogEntry(prefix + "pd_gains_ratio", this, [this]() { return rlRuntime_.pdGainsRatio(); });

  // RL variables (current observation/action, policy rate and update count are logged by mc_nn)
  logger.addLogEntry(prefix + "RL_q", this, [this]() { return rlRuntime_.q_rl(); });
  logger.addLogEntry(prefix + "RL_qZero", this, [this]() { return rlRuntime_.q_zero(); });
  logger.addLogEntry(prefix + "RL_currentActionScaled", this, [this]() { return rlRuntime_.currentActionScaled(); });
  logger.addLogEntry(prefix + "RL_actionScale", this, [this]() { return rlRuntime_.actionScale(); });
  logger.addLogEntry(prefix + "RL_command", this, [this]() { return rlRuntime_.command(); });

  // Controller state variables
  logger.addLogEntry(prefix + "useQP", this, [this]() { return rlRuntime_.useQP(); });

  // Log observation convention and phase
  logger.addLogEntry(prefix + "observationConvention", this, [this]() { return rlRuntime_.conventionName(); });
  logger.addLogEntry(prefix + "RL_phase", this, [this]() { return rlRuntime_.phase(); });

  rlRuntime_.addLogObs(ctl, prefix, this);
}

void mc_nn_SafeCBFTorquePolicyContract::addGui(mc_control::fsm::Controller & ctl, const std::vector<std::string> & category)
{
  auto sub = [&category](const std::string & name)
  {
    auto out = category;
    out.push_back(name);
    return out;
  };
  auto & gui = *ctl.gui();

  // Policy selection, reload, rates and sizes are handled by mc_nn's common GUI.
  gui.addElement(
    sub("Policy"),
    mc_rtc::gui::Label("Current policy", [this]() { return rlRuntime_.currentPolicyName(); }),
    mc_rtc::gui::Label("Current policy folder", [this]() { return rlRuntime_.currentPolicyFolder(); }),
    mc_rtc::gui::Label("Observation convention", [this]() { return rlRuntime_.conventionName(); }),
    mc_rtc::gui::Label("Observation source", [this]() { return rlRuntime_.observationSource(); }),
    mc_rtc::gui::Label("Base body", [this]() { return rlRuntime_.baseBody(); }),
    mc_rtc::gui::Label("Controlled action size", [this]() { return rlRuntime_.controlledActionSize(); }),
    mc_rtc::gui::Label("Phase", [this]() { return rlRuntime_.phase(); }));

  gui.addElement(
    sub("PD Gains"),
    mc_rtc::gui::NumberSlider(
      "PD Gains Ratio",
      [this]() { return rlRuntime_.pdGainsRatio(); },
      [this](double v) { rlRuntime_.setPDGainsRatio(v, torqueJointTask); },
      0.0,
      2.0),
    mc_rtc::gui::Label("Current kp", [this]() { return rlRuntime_.kp(); }),
    mc_rtc::gui::Label("Current kd", [this]() { return rlRuntime_.kd(); }));

  gui.addElement(
    sub("Control"),
    mc_rtc::gui::Button("Toggle QP Control", [this]() {
      rlRuntime_.setUseQP(!rlRuntime_.useQP());
    }),
    mc_rtc::gui::Label("QP Control", [this]() {
      return rlRuntime_.useQP() ? "Enforced" : "Bypassed";
    }),
    mc_rtc::gui::Button("Toggle print joint limits", [this]() {
      printLimits_ = !printLimits_;
    }),
    mc_rtc::gui::Label("Print joint limits", [this]() {
      return printLimits_ ? "Enabled" : "Disabled";
    }));

  gui.addElement(
    sub("Command"),
    mc_rtc::gui::Button(
        "Toggle Joystick Plugin",
        [this]()
        {
          toggleJoystick = !toggleJoystick;
          if(toggleJoystick)
            toggleKeyboard = false;
        }),
    mc_rtc::gui::Label(
        "Current velcity control mode",
        [this]()
        {
          if(toggleKeyboard)
            return std::string{"Keyboard"};
          if(toggleJoystick && controllerAvailable)
            return std::string{"mc_joystick_plugin"};
          return std::string{"GUI"};
        }),
    mc_rtc::gui::Button(
        "Toggle Keyboard",
        [this]()
        {
          toggleKeyboard = !toggleKeyboard;
          if(toggleKeyboard)
            toggleJoystick = false;
        }),
    mc_rtc::gui::Label(
        "Joystick plugin available",
        [this]() { return controllerAvailable ? "Yes" : "No"; }),
    mc_rtc::gui::Label("Command values", []() { return std::string(" "); }),
    mc_rtc::gui::NumberInput(
      "vx",
      [this]() { return rlRuntime_.command()(0); },
      [this](double v) { rlRuntime_.command()(0) = v; }),
    mc_rtc::gui::NumberInput(
      "vy",
      [this]() { return rlRuntime_.command()(1); },
      [this](double v) { rlRuntime_.command()(1) = v; }),
    mc_rtc::gui::NumberInput(
      "yaw_rate",
      [this]() { return rlRuntime_.command()(2); },
      [this](double v) { rlRuntime_.command()(2) = v; }),
    mc_rtc::gui::Label("Max values", []() { return std::string(" "); }),
    mc_rtc::gui::NumberInput(
        "max_vel_cmd",
        [this]() { return maxVelCmd; }, [this](double v) { maxVelCmd = v; }),
    mc_rtc::gui::NumberInput(
        "max_yaw_cmd",
        [this]() { return maxYawCmd; }, [this](double v) { maxYawCmd = v; }));
}

void mc_nn_SafeCBFTorquePolicyContract::computeLimits(mc_control::fsm::Controller & ctl)
{
  const double epsilon = 1e-5;

  mc_rbdyn::Robot & real_robot = ctl.realRobot(robotName_);

  const std::vector<std::vector<double> > & currentPos = real_robot.q();
  const std::vector<std::vector<double> > & currentVel = real_robot.alpha();
  const std::vector<std::vector<double> > & currentTau = real_robot.jointTorque();

  const std::vector<std::vector<double> > & qLimLower = real_robot.ql();
  const std::vector<std::vector<double> > & qLimUpper = real_robot.qu();

  const std::vector<std::vector<double> > & qDotLimLower = real_robot.vl();
  const std::vector<std::vector<double> > & qDotLimUpper = real_robot.vu();

  const std::vector<std::vector<double> > & tauLimLower = real_robot.tl();
  const std::vector<std::vector<double> > & tauLimUpper = real_robot.tu();

  for(size_t j = 0; j < jointNames.size(); ++j)
  {
    const std::string & joint = jointNames[j];
    const int i = real_robot.jointIndexByName(joint);
    const size_t idx = static_cast<size_t>(i);

    const double ds = dsPercent_ * (qLimUpper[idx][0] - qLimLower[idx][0]);

    const double posLimitUp = qLimUpper[idx][0] - ds;
    const double posLimitLow = qLimLower[idx][0] + ds;

    const double velLimitUp = velPercent_ * qDotLimUpper[idx][0];
    const double velLimitLow = velPercent_ * qDotLimLower[idx][0];

    const double tauLimitUp = tauLimUpper[idx][0];
    const double tauLimitLow = tauLimLower[idx][0];

    if(currentPos[idx][0] > posLimitUp + epsilon)
    {
      mc_rtc::log::warning(
        "[mc_nn_SafeCBFTorquePolicyContract] Joint {} position upper limit breached: currentPos = {}, limit = {}",
        joint,
        currentPos[idx][0],
        posLimitUp);
    }

    if(currentPos[idx][0] < posLimitLow - epsilon)
    {
      mc_rtc::log::warning(
        "[mc_nn_SafeCBFTorquePolicyContract] Joint {} position lower limit breached: currentPos = {}, limit = {}",
        joint,
        currentPos[idx][0],
        posLimitLow);
    }

    if(currentVel[idx][0] > velLimitUp + epsilon)
    {
      mc_rtc::log::warning(
        "[mc_nn_SafeCBFTorquePolicyContract] Joint {} velocity upper limit breached: currentVel = {}, limit = {}",
        joint,
        currentVel[idx][0],
        velLimitUp);
    }

    if(currentVel[idx][0] < velLimitLow - epsilon)
    {
      mc_rtc::log::warning(
        "[mc_nn_SafeCBFTorquePolicyContract] Joint {} velocity lower limit breached: currentVel = {}, limit = {}",
        joint,
        currentVel[idx][0],
        velLimitLow);
    }

    if(currentTau[idx][0] > tauLimitUp + epsilon)
    {
      mc_rtc::log::warning(
        "[mc_nn_SafeCBFTorquePolicyContract] Joint {} torque upper limit breached: currentTau = {}, limit = {}",
        joint,
        currentTau[idx][0],
        tauLimitUp);
    }

    if(currentTau[idx][0] < tauLimitLow - epsilon)
    {
      mc_rtc::log::warning(
        "[mc_nn_SafeCBFTorquePolicyContract] Joint {} torque lower limit breached: currentTau = {}, limit = {}",
        joint,
        currentTau[idx][0],
        tauLimitLow);
    }
  }
}

REGISTER_MC_NN_CONTRACT("mc_nn_SafeCBFTorquePolicyContract", mc_nn_SafeCBFTorquePolicyContract)
