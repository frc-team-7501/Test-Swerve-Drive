// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;

public class Handoff extends SubsystemBase {
  private final CANSparkMax m_HanooffMotor = new CANSparkMax(CANMapping.HANDOFF_SPARKMAX, MotorType.kBrushless);
  private static Handoff instance;

  /** Creates a new Handoff. */
  public Handoff() {

  }

  public static Handoff getInstance() {
    if (instance == null)
      instance = new Handoff();
    return instance;
  }

  public void fireHandoff(double HandoffPower) {
    m_HanooffMotor.set(HandoffPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    m_HanooffMotor.stopMotor();
  }
}
