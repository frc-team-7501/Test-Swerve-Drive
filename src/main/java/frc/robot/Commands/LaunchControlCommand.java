// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MiscMapping;
import frc.robot.Subsystems.Launcher;

public class LaunchControlCommand extends Command {
  /** Creates a new LaunchCommand. */
  private final Launcher Launcher;
  private double launchSpeedDouble = 0;
  private boolean isLauncherRunning = false;

  public LaunchControlCommand(Launcher Launcher) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Launcher = Launcher;
    addRequirements(Launcher);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // If running, stop. 
    if (isLauncherRunning) {
      isLauncherRunning = false;
      launchSpeedDouble= MiscMapping.LAUNCH_VELOCITY;
    } else {
      isLauncherRunning = true;
      launchSpeedDouble = MiscMapping.LAUNCH_VELOCITY;
    }
    
    Launcher.fireLauncher(launchSpeedDouble);
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Launcher.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
