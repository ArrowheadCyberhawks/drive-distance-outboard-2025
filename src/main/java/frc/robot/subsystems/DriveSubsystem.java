package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import java.util.List;

/**
 * Represents the robot's drive base.
 * This subsystem handles all motor control, safety features, and drive modes.
 */
public class DriveSubsystem extends SubsystemBase {
  // "Leader" motors control the output. The "Followers" just mimic them.
  private final WPI_TalonSRX m_leftLeader = new WPI_TalonSRX(DriveConstants.kLeftMotor1Port);
  private final WPI_TalonSRX m_rightLeader = new WPI_TalonSRX(DriveConstants.kRightMotor1Port);
  
  // We keep these lists to prevent the Java Garbage Collector (GC) from deleting our follower motors.
  private final List<WPI_TalonSRX> m_leftFollowers = List.of(
      new WPI_TalonSRX(DriveConstants.kLeftMotor2Port),
      new WPI_TalonSRX(DriveConstants.kLeftMotor3Port));
  private final List<WPI_TalonSRX> m_rightFollowers = List.of(
      new WPI_TalonSRX(DriveConstants.kRightMotor2Port),
      new WPI_TalonSRX(DriveConstants.kRightMotor3Port));

  // WPILib's helper class for driving. It handles the math for arcade/tank/curvature drive.
  private final DifferentialDrive m_drive = new DifferentialDrive(m_leftLeader::set, m_rightLeader::set);

  public DriveSubsystem() {
    // Register motors with the dashboard so we can see them
    SendableRegistry.addChild(m_drive, m_leftLeader);
    SendableRegistry.addChild(m_drive, m_rightLeader);

    // --- Motor Configuration ---
    // We create one config object and apply it to all motors to ensure consistency.
    var config = new TalonSRXConfiguration();
    
    // Voltage Compensation: Ensures the robot drives at the same speed whether the battery is at 12V or 10V.
    config.voltageCompSaturation = DriveConstants.kVoltageComp;
    
    // Current Limiting: CIM motors are power hungry!
    // We limit each motor to 30 Amps to prevent the main breaker from tripping (Brownout).
    config.continuousCurrentLimit = 30;
    config.peakCurrentLimit = 60;
    config.peakCurrentDuration = 100; // 100ms
    
    // Ramp Rate: Takes 0.2 seconds to go from 0% to 100% power.
    // This prevents the robot from jerking and damaging the gears or tipping over.
    config.openloopRamp = 0.2; 

    // Apply the settings to both sides
    configureSide(m_leftLeader, m_leftFollowers, config, false);
    configureSide(m_rightLeader, m_rightFollowers, config, true); // Right side is usually inverted
  }

  /**
   * Helper method to configure a "Leader" motor and all its "Followers".
   * This keeps our code DRY (Don't Repeat Yourself).
   */
  private void configureSide(WPI_TalonSRX leader, List<WPI_TalonSRX> followers, TalonSRXConfiguration config, boolean inverted) {
    // 1. Configure the Leader
    leader.configAllSettings(config);
    leader.setNeutralMode(NeutralMode.Brake); // Brake mode makes the robot stop quickly when you let go of the stick
    leader.enableVoltageCompensation(true);
    leader.enableCurrentLimit(true); // Enable the current limit we configured
    leader.setInverted(inverted);
    
    // 2. Configure the Followers
    for (var follower : followers) {
      follower.configFactoryDefault(); // Reset to factory defaults to remove any old settings
      follower.follow(leader); // IMPORTANT: This tells the motor to do exactly what the leader does
      follower.setInverted(InvertType.FollowMaster); // Follow the leader's direction
      follower.setNeutralMode(NeutralMode.Brake);
      follower.enableVoltageCompensation(true);
      follower.configVoltageCompSaturation(DriveConstants.kVoltageComp);
    }
  }

  // --- Drive Methods ---

  /** Arcade Drive: One stick for speed (fwd), one for turn (rot). Standard for most games. */
  public void arcadeDrive(double fwd, double rot) { m_drive.arcadeDrive(fwd, rot); }

  public void stopMotor() { m_drive.stopMotor(); }

  /** Tank Drive: Left stick controls left wheels, Right stick controls right wheels. */
  public void tankDrive(double left, double right) { m_drive.tankDrive(left, right); }

  /** 
   * Curvature Drive: A more advanced drive mode.
   * It handles "turning while moving" differently than "turning in place".
   * It feels much smoother at high speeds.
   */
  public void curvatureDrive(double fwd, double rot, boolean turnInPlace) { m_drive.curvatureDrive(fwd, rot, turnInPlace); }

  /** Limits the maximum speed of the robot (0.0 to 1.0). Useful for "Slow Mode". */
  public void setMaxOutput(double maxOutput) { m_drive.setMaxOutput(maxOutput); }

  /** 
   * Simple Auto Command: Drives at a set speed for a set time.
   * Since we don't have encoders, this is the best we can do for autonomous.
   * Example: driveTime(0.5, 2.0) drives at 50% speed for 2 seconds.
   */
  public Command driveTime(double speed, double seconds, double Rotation) {
    return run(() -> m_drive.arcadeDrive(speed, Rotation))
        .withTimeout(seconds)
        .andThen(() -> m_drive.stopMotor()); // Safety: Stop the motors when time is up!
  }
  //Simple command allowing control over both motors
  public Command driveRotation(double leftSpeed, double seconds, double rightSpeed) {
    return run(() -> m_drive.tankDrive(leftSpeed, rightSpeed))
        .withTimeout(seconds)
        .andThen(() -> m_drive.stopMotor()); // Safety: Stop the motors when time is up! 
    
  }
}
