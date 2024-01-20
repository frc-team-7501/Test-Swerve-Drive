// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANMapping;
import frc.robot.Constants.MiscMapping;
import frc.robot.Constants;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {
  public static final double kMaxSpeed = MiscMapping.MAXSPEED; // Power
  public static final double kMaxAngularSpeed = 0.5; // 1/2 rotation per second
  // Wheel position offsets.
  private final Translation2d m_frontLeftLocation = new Translation2d(0.292, 0.292);
  private final Translation2d m_frontRightLocation = new Translation2d(0.292, -0.292);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.292, 0.292);
  private final Translation2d m_backRightLocation = new Translation2d(-0.292, -0.292);

  private final SwerveModule m_frontLeft = new SwerveModule(CANMapping.SPARKMAX_DRIVE_FL, CANMapping.TALONSRX_TURN_FL,
      CANMapping.TURN_CANCODER_FL);
  private final SwerveModule m_frontRight = new SwerveModule(CANMapping.SPARKMAX_DRIVE_FR, CANMapping.TALONSRX_TURN_FR,
      CANMapping.TURN_CANCODER_FR);
  private final SwerveModule m_backLeft = new SwerveModule(CANMapping.SPARKMAX_DRIVE_BL, CANMapping.TALONSRX_TURN_BL,
      CANMapping.TURN_CANCODER_BL);
  private final SwerveModule m_backRight = new SwerveModule(CANMapping.SPARKMAX_DRIVE_BR, CANMapping.TALONSRX_TURN_BR,
      CANMapping.TURN_CANCODER_BR);

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

  // TODO: switch analog gyro to pigeon.
  private final PigeonIMU m_pigeonIMU = new PigeonIMU(CANMapping.PIGEON_IMU);

  private static Drivetrain instance;

  public static Drivetrain getInstance() {
    if (instance == null)
      instance = new Drivetrain();
    return instance;
  }

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
      m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      m_kinematics,
      getGyroYaw2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
      }
  );

  public Drivetrain() {
  resetYaw();
  }
 
  public void setBrakeMode(boolean enabled) {
    //TODO: Set brake mode to off
//        motorBL.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
//        motorBR.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
//        motorFL.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
//        motorFR.setNeutralMode(enabled ? NeutralMode.Brake : NeutralMode.Coast);
    }
    
    public void resetYaw() {
      m_pigeonIMU.setYaw(0);
    }

    public double getGyroYaw () {
      double [] ypr = new double [3];
      m_pigeonIMU.getYawPitchRoll(ypr);
      return ypr [0];
    }

    public Rotation2d getGyroYaw2d () {
      double [] ypr = new double [3];
      m_pigeonIMU.getYawPitchRoll(ypr);
      return Rotation2d.fromDegrees(ypr [0]);
    }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double forward, double strafe, double rotate, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates;

    // Get the y speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -m_yspeedLimiter.calculate(MathUtil.applyDeadband(forward, 0.02))
        * Drivetrain.kMaxSpeed;

    // Get the x speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -m_xspeedLimiter.calculate(MathUtil.applyDeadband(strafe, 0.02))
        * Drivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -m_rotLimiter.calculate(MathUtil.applyDeadband(rotate, 0.02))
        * Drivetrain.kMaxAngularSpeed;

    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("rotation", rot);
    SmartDashboard.putNumber("back left position", m_backRight.showRotation() % (Math.PI * 2));
    SmartDashboard.putNumber("back left turn output", m_backRight.showTurnPower());
    
    
    
    if (fieldRelative) {
      swerveModuleStates = m_kinematics.toSwerveModuleStates(
          ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getGyroYaw2d()));
    } else {
      swerveModuleStates = m_kinematics.toSwerveModuleStates(
          new ChassisSpeeds(xSpeed, ySpeed, rot));
    }

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    
    SmartDashboard.putString("desired state", swerveModuleStates[3].toString());
    
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        getGyroYaw2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        });

    SmartDashboard.putNumber("X", m_odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("Y", m_odometry.getPoseMeters().getY());
    SmartDashboard.putNumber("Heading", m_odometry.getPoseMeters().getRotation().getDegrees());
  }
}
