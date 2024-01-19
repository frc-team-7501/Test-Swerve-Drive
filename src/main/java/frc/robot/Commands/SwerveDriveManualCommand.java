// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Drivetrain;

public class SwerveDriveManualCommand extends Command {
  /** Creates a new SwerveDriveManualCommand. */
  private final Drivetrain driveTrain;
  private final DoubleSupplier forwardSupplier;
  private final DoubleSupplier strafeSupplier;
  private final DoubleSupplier rotateSupplier;
  private final BooleanSupplier fieldRelative;

  public SwerveDriveManualCommand( 
    final Drivetrain driveTrain, 
    DoubleSupplier forwardSupplier, 
    DoubleSupplier strafeSupplier,
    DoubleSupplier rotateSupplier,
    BooleanSupplier fieldRelative) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.forwardSupplier = forwardSupplier;
    this.strafeSupplier = strafeSupplier;
    this.rotateSupplier = rotateSupplier;
    this.fieldRelative = fieldRelative;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  //TODO: create a stop in Drivetrain
    //  driveTrain.stop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveTrain.drive(
      forwardSupplier.getAsDouble(), 
      strafeSupplier.getAsDouble(), 
      rotateSupplier.getAsDouble(), 
      fieldRelative.getAsBoolean()
      );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
