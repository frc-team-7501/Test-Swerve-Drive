// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Handoff;
import frc.robot.Subsystems.Intake;

public class HandoffControlCommand extends Command {
  /** Creates a new LaunchCommand. */
  private final Handoff Handoff;
  private double handoffSpeed;

  public HandoffControlCommand(Handoff handoff, Double handoffSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.Handoff = handoff;
    this.handoffSpeed = handoffSpeed;
    addRequirements(handoff);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Handoff.fireHandoff(handoffSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Handoff.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
