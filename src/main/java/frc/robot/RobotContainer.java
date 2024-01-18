// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Subsystems.*;
import frc.robot.Commands.*;
import frc.robot.Constants.*;

public class RobotContainer {
  private final XboxController m_Xbox = new XboxController(0);
  private final Joystick m_Joystick= new Joystick (1);

  //create subsystems
  private final Drivetrain driveTrain = Drivetrain.getInstance();

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
      () -> MiscMapping.FIELD_RELATIVE
      );

      public RobotContainer() {
        driveTrain.setDefaultCommand(swerveDriveManualCommand);
      }
      
      public void teleopInit() {
        driveTrain.setBrakeMode(MiscMapping.BRAKE_OFF);
      }

      public void autonomousInit() {
        driveTrain.setBrakeMode(MiscMapping.BRAKE_ON);
      }

      public Command getAutonomousCommand() {
        // [MAIN AUTONS]
         return DefaultAuton; // Literally just a wait command to satisfy WPILIB.
    }
}
