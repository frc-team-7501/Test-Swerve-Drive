// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.Autonomous;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Launcher;

public class AutonLauncherCommand extends Command {
  private final Launcher launcher;
  private final double velocity;
  /** Creates a new AutonLauncherCommand. */
  public AutonLauncherCommand(final Launcher launcher, final double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    SmartDashboard.putNumber("auton velocity", velocity);
    addRequirements(launcher);
    this.launcher = launcher;
    this.velocity = velocity;
    launcher.fireLauncher(velocity);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if ((launcher.getLauncherVelocity() > velocity - 10) || (launcher.getLauncherVelocity() < velocity + 10)){
      return true;
    }
    return false;
  }
}
