// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.*;
import frc.robot.utils.ExtendedXboxController;
import frc.robot.Commands.*;
import frc.robot.Constants.*;

public class RobotContainer {
  private final ExtendedXboxController m_Xbox = new ExtendedXboxController(ControllerMapping.XBOX);
  private final Joystick m_Joystick = new Joystick(ControllerMapping.JOYSTICK);

  // create subsystems
  private final Drivetrain driveTrain = Drivetrain.getInstance();
  private final Launcher launcher = Launcher.getInstance();

  ////////////////////////////////
  // #region [ AUTON COMMANDS ]
  // #region Placeholder
  // Auton placeholder
  private final Command DefaultAuton = new SequentialCommandGroup(
      new WaitCommand(7.501));
  // #endregion
  // #endregion

  // Create commands
  private final Command swerveDriveManualCommand = new SwerveDriveManualCommand(
      driveTrain,
      () -> m_Xbox.getLeftY(),
      () -> m_Xbox.getLeftX(),
      () -> m_Xbox.getRightX(),
      () -> MiscMapping.FIELD_RELATIVE);

  private final InstantCommand ResetGyroYawInstantCommand = new ResetGyroYawInstantCommand(
      driveTrain);
  
  //private final Command launchControlCommand = new LaunchControlCommand(launcher, 500.0);

  public RobotContainer() {
    
    configureButtonBindings();

    driveTrain.setDefaultCommand(swerveDriveManualCommand);
    //launcher.setDefaultCommand(LaunchControlCommand);
    
  }

  private void configureButtonBindings() {
    // Back button on the drive controller resets gyroscope.
    m_Xbox.b_Back().onTrue(new ResetGyroYawInstantCommand(driveTrain));
    m_Xbox.b_A().onTrue(new LaunchControlCommand(launcher, MiscMapping.LAUNCH_VELOCITY));
    m_Xbox.b_A().onFalse(new LaunchControlCommand(launcher, 0.0));
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
