// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.SwerveModule;
import frc.robot.Constants.CANMapping;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 1.0; // Power
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
//TODO: Put wheel positions here 
  private final Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
  private final Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
  private final Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);

  private final SwerveModule m_frontLeft = new SwerveModule(CANMapping.SPARKMAX_DRIVE_FL, CANMapping.TALONSRX_TURN_FL, CANMapping.TURN_CANCODER_FL);
  private final SwerveModule m_frontRight = new SwerveModule(CANMapping.SPARKMAX_DRIVE_FR, CANMapping.TALONSRX_TURN_FR, CANMapping.TURN_CANCODER_FR);
  private final SwerveModule m_backLeft = new SwerveModule(CANMapping.SPARKMAX_DRIVE_BL, CANMapping.TALONSRX_TURN_BL, CANMapping.TURN_CANCODER_BL);
  private final SwerveModule m_backRight = new SwerveModule(CANMapping.SPARKMAX_DRIVE_BR, CANMapping.TALONSRX_TURN_BR, CANMapping.TURN_CANCODER_BR);
  //TODO: switch analog gyro to pigeon.
  private final AnalogGyro m_gyro = new AnalogGyro(0);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          m_gyro.getRotation2d(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Drivetrain() {
    m_gyro.reset();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates;
    if (fieldRelative) {
      swerveModuleStates = m_kinematics.toSwerveModuleStates(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, m_gyro.getRotation2d()));
    } else {
      swerveModuleStates = m_kinematics.toSwerveModuleStates(
        new ChassisSpeeds(xSpeed, ySpeed, rot));
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);

    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });

    SmartDashboard.putNumber("X",       m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y",       m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Heading", m_odometry.getPoseMeters().getRotation().getDegrees());
  }
}
