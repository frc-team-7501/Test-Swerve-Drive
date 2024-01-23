// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;

public class Launcher extends SubsystemBase {
  final TalonFX m_LaunchMotor = new TalonFX(CANMapping.LAUNCH_TALON);
  private static Launcher instance;
  
  /** Creates a new Launcher. */
  public Launcher() {}

 public static Launcher getInstance() {
  if (instance == null)
      instance = new Launcher();
    return instance;
 }
  
 public void fireLauncher(double speed) {
  m_LaunchMotor.set(speed);
 }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    m_LaunchMotor.stopMotor();
  }
}
