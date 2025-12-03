// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  // The motors on the left side of the drive.
  private final WPI_TalonSRX m_leftLeader =
      new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);

  private final WPI_TalonSRX m_leftFollower1 =
      new WPI_TalonSRX(DriveConstants.kLeftMotor2Port);

  private final WPI_TalonSRX m_leftFollower2 =
      new WPI_TalonSRX(DriveConstants.kLeftMotor3Port);

  // The motors on the right side of the drive.
  private final WPI_TalonSRX m_rightLeader =
      new WPI_TalonSRX(DriveConstants.kRightMotor1Port);

  private final WPI_TalonSRX m_rightFollower1 =
      new WPI_TalonSRX(DriveConstants.kRightMotor2Port);

  private final WPI_TalonSRX m_rightFollower2 =
      new WPI_TalonSRX(DriveConstants.kRightMotor3Port);

  // The feedforward controller.
  private final SimpleMotorFeedforward m_feedforward =
      new SimpleMotorFeedforward(
          DriveConstants.ksVolts,
          DriveConstants.kvVoltSecondsPerMeter,
          DriveConstants.kaVoltSecondsSquaredPerMeter);

  // The robot's drive
  private final DifferentialDrive m_drive =
      new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);

  // The trapezoid profile
  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              DriveConstants.kMaxSpeedMetersPerSecond,
              DriveConstants.kMaxAccelerationMetersPerSecondSquared));

  // The timer
  private final Timer m_timer = new Timer();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    SendableRegistry.addChild(m_drive, m_leftLeader);
    SendableRegistry.addChild(m_drive, m_rightLeader);

    // Create a configuration object for the leaders
    TalonSRXConfiguration leaderConfig = new TalonSRXConfiguration();

    // Common Config
    leaderConfig.slot0.kP = DriveConstants.kp;
    leaderConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    leaderConfig.voltageCompSaturation = DriveConstants.kVoltageComp;
    
    // 2. Cook the Leaders
    configureLeader(m_leftLeader, leaderConfig, false);
    configureLeader(m_rightLeader, leaderConfig, true);

    // Configure Followers
    configureFollower(m_leftFollower1, m_leftLeader);
    configureFollower(m_leftFollower2, m_leftLeader);
    configureFollower(m_rightFollower1, m_rightLeader);
    configureFollower(m_rightFollower2, m_rightLeader);
  }

  private void configureLeader(WPI_TalonSRX leader, TalonSRXConfiguration config, boolean inverted) {
    leader.configAllSettings(config);
    leader.setNeutralMode(NeutralMode.Brake);
    leader.enableVoltageCompensation(true);
    leader.setInverted(inverted);
    
    // Current Limiting (Prevent Brownouts)
    // 40A continuous, 60A peak for 100ms
    leader.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 60, 0.1));
  }

  private void configureFollower(WPI_TalonSRX follower, WPI_TalonSRX leader) {
    follower.configFactoryDefault();
    follower.follow(leader);
    follower.setInverted(InvertType.FollowMaster);
    follower.setNeutralMode(NeutralMode.Brake);
    follower.configVoltageCompSaturation(DriveConstants.kVoltageComp);
    follower.enableVoltageCompensation(true);
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    m_drive.arcadeDrive(fwd, rot);
  }

  /**
   * Drives the robot using tank controls.
   *
   * @param left the commanded left side movement
   * @param right the commanded right side movement
   */
  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  /**
   * Drives the robot using curvature controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   * @param allowTurnInPlace whether to allow turning in place
   */
  public void curvatureDrive(double fwd, double rot, boolean allowTurnInPlace) {
    m_drive.curvatureDrive(fwd, rot, allowTurnInPlace);
  }

  /**
   * Attempts to follow the given drive states using offboard PID.
   *
   * @param currentLeft The current left wheel state.
   * @param currentRight The current right wheel state.
   * @param nextLeft The next left wheel state.
   * @param nextRight The next right wheel state.
   */
  public void setDriveStates(
      TrapezoidProfile.State currentLeft,
      TrapezoidProfile.State currentRight,
      TrapezoidProfile.State nextLeft,
      TrapezoidProfile.State nextRight) {
    // Feedforward is divided by battery voltage to normalize it to [-1, 1]
    m_leftLeader.set(
        ControlMode.Position,
        currentLeft.position / DriveConstants.kEncoderDistancePerPulse,
        DemandType.ArbitraryFeedForward,
        m_feedforward.calculateWithVelocities(currentLeft.velocity, nextLeft.velocity)
            / DriveConstants.kVoltageComp);
    m_rightLeader.set(
        ControlMode.Position,
        currentRight.position / DriveConstants.kEncoderDistancePerPulse,
        DemandType.ArbitraryFeedForward,
        m_feedforward.calculateWithVelocities(currentLeft.velocity, nextLeft.velocity)
            / DriveConstants.kVoltageComp);
  }

  /**
   * Returns the left encoder distance.
   *
   * @return the left encoder distance
   */
  public double getLeftEncoderDistance() {
    return m_leftLeader.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse;
  }

  /**
   * Returns the right encoder distance.
   *
   * @return the right encoder distance
   */
  public double getRightEncoderDistance() {
    return m_rightLeader.getSelectedSensorPosition() * DriveConstants.kEncoderDistancePerPulse;
  }

  /** Resets the drive encoders. */
  public void resetEncoders() {
    m_leftLeader.setSelectedSensorPosition(0);
    m_rightLeader.setSelectedSensorPosition(0);
  }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

  /**
   * Creates a command to drive forward a specified distance using a motion profile.
   *
   * @param distance The distance to drive forward.
   * @return A command.
   */
  public Command profiledDriveDistance(double distance) {
    return startRun(
            () -> {
              // Restart timer so profile setpoints start at the beginning
              m_timer.restart();
              resetEncoders();
            },
            () -> {
              // Current state never changes, so we need to use a timer to get the setpoints we need
              // to be at
              var currentTime = m_timer.get();
              var currentSetpoint =
                  m_profile.calculate(currentTime, new State(), new State(distance, 0));
              var nextSetpoint =
                  m_profile.calculate(
                      currentTime + DriveConstants.kDt, new State(), new State(distance, 0));
              setDriveStates(currentSetpoint, currentSetpoint, nextSetpoint, nextSetpoint);
            })
        .until(() -> m_profile.isFinished(0));
  }

  private double m_initialLeftDistance;
  private double m_initialRightDistance;

  /**
   * Creates a command to drive forward a specified distance using a motion profile without
   * resetting the encoders.
   *
   * @param distance The distance to drive forward.
   * @return A command.
   */
  public Command dynamicProfiledDriveDistance(double distance) {
    return startRun(
            () -> {
              // Restart timer so profile setpoints start at the beginning
              m_timer.restart();
              // Store distance so we know the target distance for each encoder
              m_initialLeftDistance = getLeftEncoderDistance();
              m_initialRightDistance = getRightEncoderDistance();
            },
            () -> {
              // Current state never changes for the duration of the command, so we need to use a
              // timer to get the setpoints we need to be at
              var currentTime = m_timer.get();
              var currentLeftSetpoint =
                  m_profile.calculate(
                      currentTime,
                      new State(m_initialLeftDistance, 0),
                      new State(m_initialLeftDistance + distance, 0));
              var currentRightSetpoint =
                  m_profile.calculate(
                      currentTime,
                      new State(m_initialRightDistance, 0),
                      new State(m_initialRightDistance + distance, 0));
              var nextLeftSetpoint =
                  m_profile.calculate(
                      currentTime + DriveConstants.kDt,
                      new State(m_initialLeftDistance, 0),
                      new State(m_initialLeftDistance + distance, 0));
              var nextRightSetpoint =
                  m_profile.calculate(
                      currentTime + DriveConstants.kDt,
                      new State(m_initialRightDistance, 0),
                      new State(m_initialRightDistance + distance, 0));
              setDriveStates(
                  currentLeftSetpoint, currentRightSetpoint, nextLeftSetpoint, nextRightSetpoint);
            })
        .until(() -> m_profile.isFinished(0));
  }
}
