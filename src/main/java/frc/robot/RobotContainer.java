// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.*;
import frc.robot.utils.ExtendedJoystick;
import frc.robot.utils.ExtendedXboxController;
import frc.robot.Commands.*;
import frc.robot.Commands.Autonomous.AutonLauncherCommand;
import frc.robot.Constants.ControllerMapping;
import frc.robot.Constants.MiscMapping;

public class RobotContainer {
  private final ExtendedXboxController m_Xbox = new ExtendedXboxController(ControllerMapping.XBOX);
  private final ExtendedJoystick m_Joystick = new ExtendedJoystick(ControllerMapping.JOYSTICK);

  // create subsystems
  private final Drivetrain driveTrain = Drivetrain.getInstance();
  private final Launcher launcher = Launcher.getInstance();
  private final Intake intake = Intake.getInstance();
  private final Handoff handoff = Handoff.getInstance();

  public double XPosition;
  public double YPosition;
  public double ZPosition;

  ////////////////////////////////
  // #region [ AUTON COMMANDS ]
  // #region Placeholder
  // Auton placeholder
  private final Command DefaultAuton = new SequentialCommandGroup(  
      new AutonLauncherCommand(launcher, 0),
      new WaitCommand(0.1),
      new AutonLauncherCommand(launcher, MiscMapping.LAUNCH_VELOCITY),
      new WaitCommand(5.0));
      //new AutonLauncherCommand(launcher,0));

  // #endregion
  // #endregion

  // Create commands
  // private final Command swerveDriveManualCommand = new
  // SwerveDriveManualCommand(
  // driveTrain,
  // () -> XPosition,
  // () -> YPosition,
  // () -> ZPosition,
  // () -> MiscMapping.FIELD_RELATIVE);

  private final Command swerveDriveManualCommand = new SwerveDriveManualCommand(
      driveTrain,
      () -> m_Xbox.getLeftY(),
      () -> m_Xbox.getLeftX(),
      () -> m_Xbox.getRightX(),
      () -> MiscMapping.FIELD_RELATIVE);

  // private final InstantCommand ResetGyroYawInstantCommand = new
  // ResetGyroYawInstantCommand(
  // driveTrain);

  public RobotContainer() {
    configureButtonBindings();
    driveTrain.setDefaultCommand(swerveDriveManualCommand);
  }

  private void configureButtonBindings() {
    // Back button on the drive controller resets gyroscope.
    m_Xbox.b_Back().onTrue(new ResetGyroYawInstantCommand(driveTrain));

    m_Xbox.b_B().onTrue(new LaunchControlCommand(launcher, MiscMapping.LAUNCH_VELOCITY));
    m_Xbox.b_B().onFalse(new LaunchControlCommand(launcher, 0.0));

    m_Xbox.b_A().onTrue(new IntakeControlCommand(intake, MiscMapping.INTAKE_VELOCITY));
    m_Xbox.b_A().onFalse(new IntakeControlCommand(intake, 0.0));

    m_Xbox.b_X().onTrue(new HandoffControlCommand(handoff, MiscMapping.HANDOFF_SPEED));
    m_Xbox.b_X().onFalse(new HandoffControlCommand(handoff, 0.0));
  }

  public void teleopInit() {
    driveTrain.setBrakeMode(MiscMapping.BRAKE_OFF);
  }

  public void autonomousInit() {
    driveTrain.setBrakeMode(MiscMapping.BRAKE_ON);
  }

  public Command getAutonomousCommand() {
    // [ MAIN AUTONS ]
    return DefaultAuton; // Just a wait command to satisfy WPILIB.
  }
}
