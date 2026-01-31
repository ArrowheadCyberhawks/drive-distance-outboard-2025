// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  public final DriveSubsystem m_robotDrive = new DriveSubsystem();

  // The driver's controller
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);

  private final SparkMax testMotor = new SparkMax(DriveConstants.kTestMotorPort, MotorType.kBrushless);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    // Set the default drive command to curvature drive
    m_robotDrive.setDefaultCommand(
        // Curvature drive with forward/backward controlled by the left Y,
        // and turning controlled by the right X.
        // Right bumper is used for "quick turn" (turning in place).
        Commands.run(
            () ->
                m_robotDrive.curvatureDrive(
                    -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                    -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                    m_driverController.rightBumper().getAsBoolean()),
            m_robotDrive));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Drive at half speed when the left bumper is held
    m_driverController
        .leftBumper()
        .whileTrue(
            Commands.startEnd(
                () -> m_robotDrive.setMaxOutput(0.5), () -> m_robotDrive.setMaxOutput(1)));

    // Drive forward at 50% speed for 2 seconds when the 'A' button is pressed
    m_driverController.a().onTrue(m_robotDrive.driveTime(0.5, 2, 0));

    // Drive backward at 50% speed for 2 seconds when the 'B' button is pressed
    m_driverController.b().onTrue(m_robotDrive.driveTime(-0.5, 2.0, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
    m_robotDrive.driveRotation(0.2, 1.5, 0.2, false),
    m_robotDrive.driveRotation(0.4, 1, 0.133, false),
    m_robotDrive.driveRotation(0.133, 1.35, 0.4, false),
    m_robotDrive.driveRotation(0.2, 3,0.2, false)
    );
    /*m_robotContainer.m_robotDrive.driveRotation(0.133, 5, 0.283, false).schedule(); 
    These values run a radius turn of around 7ft diameter*/
  }
}
