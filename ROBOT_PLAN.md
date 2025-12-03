# Robot Completion Plan: Drive Base Only

Since we currently have a robust **Differential Drive** base with **Talon SRX** controllers and **Curvature Drive**, our immediate goal is to tune this base to perfection before adding manipulators.

## Phase 1: Hardware Verification (The "Smoke Test")
Before running any complex code, we must verify the physical setup.
- [x] **CAN IDs**: Confirm `Constants.java` matches the physical CAN IDs of your Talon SRXs.
- [ ] **Motor Direction**:
    - Put the robot on blocks.
    - Drive "Forward". Do all wheels spin forward?
    - If not, adjust `setInverted` in `DriveSubsystem.java`.
- [ ] **Encoder Phase**:
    - Drive "Forward". Do the encoder values in SmartDashboard/Shuffleboard *increase*?
    - If they decrease, the sensor phase is wrong. (Note: `WPI_TalonSRX` handles this automatically if `setInverted` is correct, but verify it).
- [ ] **Encoder Resolution**:
    - Mark the wheel. Rotate it exactly 1 full rotation.
    - Does the code report exactly `0.478` meters (approx 1.57 feet)?
    - If not, check `kEncoderCPR` (4096 for MagEncoder) and `kWheelDiameterMeters` (0.1524 for 6").

## Phase 2: Characterization (SysId)
To make the **Motion Profiling** (smooth auto) work, we need to know the physics of your specific robot.
- [ ] **Install SysId**: It comes with WPILib tools.
- [ ] **Configure SysId**: Select "Talon SRX", "Encoder", "Romio/Rio".
- [ ] **Run Tests**:
    - Quasistatic (Slow ramp) -> Finds Friction (`kS`) and Velocity (`kV`).
    - Dynamic (Fast step) -> Finds Acceleration (`kA`).
- [ ] **Update Constants**: Replace the dummy values in `Constants.DriveConstants` with your SysId results.

## Phase 3: PID Tuning (Onboard Talon SRX)
We are running PID on the Talon itself. We need to tune `kP`.
- [ ] **Set `kF` (Feedforward)**: Since we are using the RIO for feedforward (`SimpleMotorFeedforward`), we can leave the Talon's `kF` at 0, OR calculate it as `1023 / MaxNativeVelocity`.
- [ ] **Tune `kP`**:
    - Command the robot to go 1 meter.
    - If it doesn't reach 1 meter, increase `kP`.
    - If it oscillates (shakes) at the end, decrease `kP`.
    - Start small (e.g., 0.1) and double it until it oscillates, then back off.

## Phase 4: Autonomous Routine
We need a plan for the first 15 seconds of the match.
- [ ] **Simple Auto**: "Drive Forward 2 Meters" (Already implemented as `profiledDriveDistance`).
- [ ] **Complex Auto**:
    - Do we need to turn?
    - Do we need to follow a curved path?
    - *Recommendation*: If you need curved paths, look into **PathPlanner** or **WPILib Trajectory**.

## Phase 5: Driver Comfort
Refine the "feel" of the robot.
- [x] **Deadband**: Implemented (0.05) in `RobotContainer` to prevent drift.
- [ ] **Slew Rate Limiting**: Does the robot tip over if you slam the stick forward? Add a `SlewRateLimiter` in `RobotContainer`.
- [ ] **Quick Turn Sensitivity**: Is the "turn in place" too fast? Scale the input in `curvatureDrive`.

## Phase 6: Future Manipulator
When you are ready to add a mechanism (Arm, Shooter, Intake):
1.  Create a new Subsystem (e.g., `ArmSubsystem`).
2.  Define its Motors and Sensors in `Constants`.
3.  Create Commands (e.g., `MoveArmToAmp`).
4.  Bind buttons in `RobotContainer`.

## Driver Controls (Xbox Controller)
The robot is configured for **Curvature Drive**, which offers smooth high-speed driving and agile low-speed maneuvering.

| Input | Action | Notes |
| :--- | :--- | :--- |
| **Left Stick Y** | **Throttle** (Forward/Reverse) | Includes 5% deadband to prevent drift. |
| **Right Stick X** | **Turn** (Left/Right) | Includes 5% deadband. Controls curvature (turn rate) when moving. |
| **Right Bumper** | **Quick Turn** (Hold) | Allows the robot to spin in place like a tank. |
| **Left Bumper** | **Slow Mode** (Hold) | Caps speed at 50% for precision alignment. |
| **A Button** | **Auto Move** (3 Meters) | Resets encoders and drives 3m using a smooth motion profile. |
| **B Button** | **Auto Move** (Relative) | Drives 3m relative to current position (good for chaining moves). |

