<p align="center">
  <a >
    <img src="doc/logo.png" alt="logo" width="300">
  </a>
</p>

---

# mc_nn_SafeCBFTorquePolicyContract: torque-level CBF-QP for RL policies in mc_rtc

**mc_nn_SafeCBFTorquePolicyContract** connects reinforcement learning (RL) policies to
[mc_rtc] through [mc_nn]. It builds observations, converts policy actions into
joint targets and computes PD torque targets, which a Control Barrier Function
Quadratic Program (CBF-QP) filters before commanding the robot.

mc_nn handles model discovery, scheduling, policy lifecycle, GUI controls and
common logs; this contract provides the robot interface and CBF-QP integration.
It is built as an
[external mc_nn contract](https://github.com/Noceo200/mc_nn/tree/main/contracts/README.md#external-contracts)
against an installed mc_nn.

The CBF-QP enforces configured physical and safety constraints, including:

* Joint position limits
* Joint velocity limits
* Torque limits
* Self-collision avoidance

The contract also supports direct torque execution with `use_QP: false`;
in that mode, the CBF-QP constraints are not enforced.

Further details are available in:

[*Safe Execution of RL Policies via Acceleration-based CBF-QP Constraint Enforcement for Real-World Robotic Deployments*](https://hal.science/hal-05362571)

Currently only ONNX format policies are supported.

Robot-specific RL-QP implementations are available for:
- [H1](https://github.com/Alhuuin/h1_rl_qp_controller)
- [HRP5P](https://github.com/bastien-muraccioli/hrp5p_rl_qp_controller)

If the target robot is already supported, please refer to the corresponding repository above. Otherwise, this contract is intended as a starting point to integrate additional robots.

## Documentation

Documentation of the RL-QP architecture, observation system, policy configuration
and robot adaptation is available at:

**https://alhuuin.github.io/rl-qp-controller.github.io/**

Use those guides for RL-QP concepts and robot adaptation. For deployment through
mc_nn, follow the build, policy and host configuration instructions below.

## Requirements

- The mc_rtc/TVM versions providing `mc_tasks::TorqueJointTask`, the CBF
  `DynamicsConstraint` (5-parameter damper) and
  `CollisionsConstraint::setCollisionsDampers`. Build mc_rtc with them; otherwise
  CMake skips the contract (`mc_nn contract mc_nn_SafeCBFTorquePolicyContract: skipped ...`).
- [mc_nn], built and installed.
- Optional: `mc_joystick_plugin` for joystick commands (detected at build time;
  keyboard and GUI commands always work).
- A host controller that (both done by mc_nn's `MCNN` controller; see
  [Host controller requirements](https://github.com/Noceo200/mc_nn/tree/main/contracts/README.md#host-controller-requirements) for other hosts):
  - calls mc_nn's post-solve step (`"MCNN::AfterSolve"`), used when the QP is
    bypassed (`use_QP: false`). mc_nn refuses to start the policy otherwise;
  - runs with `FeedbackType: ClosedLoopIntegrateReal` (a warning is printed otherwise).
- The host FSM must **not** declare its own `dynamics` constraint: the contract
  installs the CBF dynamics constraint while it runs and restores the previous
  one when it stops. Keep `collisions` and the observer pipeline in the host
  configuration (see [Host configuration](#host-configuration)).

## Layout

- `mc_nn_SafeCBFTorquePolicyContract.h/.cpp`: constraints, torque task, QP bypass, commands, GUI and logs.
- `include/observation/`, `src/observation/`: reusable observation implementations.
- `include/policy/`, `src/policy/`: policy configuration and runtime state (`RLPolicyRuntime`).
- `scripts/inspect_policy.py`: print an ONNX model's inputs and outputs.
- `policies/`: example policies (their `.onnx` files are empty placeholders: replace them with your exported models).
  - `minimalExample/`: smallest explicitly configured policy accepted by the current parser.
  - `fullExample/`: exhaustive reference for every currently accepted policy and observation fields.
  - `conventions.yaml`: placeholder joint groups, training orders, aliases and observation defaults.

## Required robot adaptation

The source marks adaptation points with `TODO(robot)`. Search them before attempting to run the contract:

```bash
grep -rn "TODO(robot)" .
```

## Deploy your own policy

Each policy is a directory holding everything the contract needs to load, run and use the policy.
Directories can live anywhere; mc_nn finds them through `models_dirs`.

1. Create a new directory next to a `conventions.yaml` (or set `conventions_file`).
2. Add the exported model in ONNX format. Its file name (without `.onnx`), plus any configured `prefix`, is the policy id in mc_nn.
3. Set the action and controlled-joint groups, provide training-consistent `kp` and `kd`, and add any required action scaling, default pose, phase period or command parameters in `policy.yaml`.
4. Reproduce the exact observation order and history in `observations.yaml`.

The model is selected by mc_nn; the `onnx` field in `policy.yaml` is ignored.
Its `name` field is used as a display name in the contract GUI.

Then select the policies in an mc_nn state:

```yaml
states:
  RLQPPolicies:
    base: RunNNBase
    # ---------------- Global RunNN configuration
    models_dirs: ["<install prefix>/share/mc_nn_SafeCBFTorquePolicyContract/policies"] # or your own policy folders
    contracts_dirs: ["<install prefix>/lib/mc_nn_contracts"]      # where this contract library is installed
    preload: false
    verbose: 1
    gui: true
    logs: true
    active_policies: [minimalExample] # policy ids launched when the state starts
    # ---------------- Policy configurations
    policies:
      - onnx: ["*"]     # every policy under models_dirs, switchable from the GUI
        prefix: ""
        contract: mc_nn_SafeCBFTorquePolicyContract
        # -- Common contract configuration (read by RunNN)
        policy_hz: 0      # 0 = control.period_s / control.frequency_hz from policy.yaml
        timeout: 0.0      # never times out
        blocking: true
        exclusive: true   # drives every joint in torque: pauses other policies (contract default)
        device: auto      # cpu | cuda | auto
        # -- Contract-specific configuration (mc_nn_SafeCBFTorquePolicyContract)
        robot:
          base_body: base_link            # TODO(robot): body used by floating-base observations
          observation_source: realRobot   # realRobot is usually appropriate on hardware; robot can be useful in simulation
        conventions_file: ""              # "" = conventions.yaml in the parent of each policy folder
```

## Build

Build and install [mc_nn] first, then:

```bash
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo -Dmc_nn_DIR=<mc_nn prefix>/lib/cmake/mc_nn
make
make install
```

Use `contract: mc_nn_SafeCBFTorquePolicyContract` to select this installed
contract at runtime. `find_package(mc_nn)` remains the build dependency; the
contract itself is loaded from `contracts_dirs`.

`mc_nn_DIR` can be omitted when mc_nn is installed in a default location, or
built in `../mc_nn/install` (mcrtc workspace layout). Add
`-DMC_RTC_HONOR_INSTALL_PREFIX=ON -DCMAKE_INSTALL_PREFIX=<prefix>` to install
elsewhere than next to mc_rtc. CMake prints
`mc_nn contract mc_nn_SafeCBFTorquePolicyContract: enabled` when the required mc_rtc
version is found. The library is installed in `<install prefix>/lib/mc_nn_contracts`:
list that folder in RunNN's `contracts_dirs`. Policy examples are installed under
`<install prefix>/share/mc_nn_SafeCBFTorquePolicyContract/policies`. Rebuild this
contract when mc_nn changes.

## Host configuration

Configure solver feedback, environment robots, collisions and observers in the
host controller's robot configuration, for example
`~/.config/mc_rtc/controllers/MCNN/<robot>.yaml`:

```yaml
FeedbackType: ClosedLoopIntegrateReal
robots:
  ground:
    module: env/ground
collisions:
  - type: collision
    useMinimal: true
# Constraint examples. Keep only constraints supported by your robot and application.
# Do not add a `dynamics` constraint: the contract installs the CBF one.
# constraints:
#   - type: contact
#     contactType: acceleration
#   - type: compoundJoint

# TODO(robot): adjust the ObserverPipeline
ObserverPipelines:
  name: MainPipeline
  gui: true
  log: true
  observers:
    - type: Encoder
      config:
        velocity: encoderVelocities
```

## Runtime controls and cleanup

Use mc_nn's ready-policies dropdown and Launch, Pause/Play and Reload controls
to manage policies. Common status and rates appear in the per-policy GUI;
contract-specific controls are under
`MCNN / <state> / <policy id> / mc_nn_SafeCBFTorquePolicyContract`.
Logs use the `MCNN_<policy id>_*` prefix, with observations, actions and rates
logged by mc_nn.

When the contract stops (pause, remove, end of state), it removes its torque
task, restores the host's posture task, dynamics constraint and `ControlMode`.
The self-collision dampers keep the contract's values (mc_rtc has no getter).

## Control flow summary

```
Every controller timestep (physics_step_size):
├── update(): commands (joystick/keyboard), limit checks, phase, torque target = q_rl
├── If mc_nn's policy period elapsed (step()):
│   ├── observation = getCurrentObservation()
│   ├── action      = rlPolicy->predict(observation)
│   └── q_rl        = action * actionScale + q_zero
│
├── τ = Kp*(q_rl - q) - Kd*q̇
│
├── useQP=true:  τ → TorqueJointTask → CBF-QP → robot
└── useQP=false: τ → robot (direct, afterSolve())
```

[mc_rtc]: https://jrl-umi3218.github.io/mc_rtc/
[mc_nn]: https://github.com/Noceo200/mc_nn
