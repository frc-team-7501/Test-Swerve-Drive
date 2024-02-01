// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;
import frc.robot.Constants.TalonMapping;

public class Launcher extends SubsystemBase {
  private final TalonFX m_LaunchMotor = new TalonFX(CANMapping.LAUNCH_TALON);
  private static Launcher instance;

  private final VelocityVoltage m_voltageVelocity = new VelocityVoltage(0);

  /** Creates a new Launcher. */
  public Launcher() {
    TalonFXConfiguration configs = new TalonFXConfiguration();

    /*
     * Voltage-based velocity requires a feed forward to account for the back-emf of
     * the motor
     */
    configs.Slot0.kP = TalonMapping.PID_P; // An error of 1 rotation per second results in 2V output
    configs.Slot0.kI = TalonMapping.PID_I; // An error of 1 rotation per second increases output by 0.5V every second
    configs.Slot0.kD = TalonMapping.PID_D; // A change of 1 rotation per second squared results in 0.01 volts output
    configs.Slot0.kV = TalonMapping.PID_V; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12
                             // volts / Rotation per second
    // Peak output of 11 volts
    configs.Voltage.PeakForwardVoltage = TalonMapping.PEAK_VOLTAGE;
    configs.Voltage.PeakReverseVoltage = -TalonMapping.PEAK_VOLTAGE;
    // Peak output of 80 amps
    configs.TorqueCurrent.PeakForwardTorqueCurrent = TalonMapping.PEAK_AMPERAGE;
    configs.TorqueCurrent.PeakReverseTorqueCurrent = -TalonMapping.PEAK_AMPERAGE;
    /* Retry config apply up to 5 times, report if failure */
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = m_LaunchMotor.getConfigurator().apply(configs);
      SmartDashboard.putNumber("i", i);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      
      System.out.println("Could not apply configs, error code: " + status.toString());
    }

  }

  public static Launcher getInstance() {
    if (instance == null)
      instance = new Launcher();
    return instance;
  }

  public void fireLauncher(double velocity) {
    m_LaunchMotor.setControl(m_voltageVelocity.withVelocity(velocity));
    SmartDashboard.putNumber("velocity", velocity);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void stop() {
    m_LaunchMotor.stopMotor();
  }
}
