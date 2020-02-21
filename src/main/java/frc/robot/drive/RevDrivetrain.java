/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drive;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Gains.*;

import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class RevDrivetrain extends SubsystemBase {

  CANSparkMax LFrontWheel = new CANSparkMax(kLeftFrontPort, MotorType.kBrushless);
  CANSparkMax RFrontWheel = new CANSparkMax(kRightFrontPort, MotorType.kBrushless);

  CANSparkMax LRearWheel = new CANSparkMax(kLeftRearPort, MotorType.kBrushless);
  CANSparkMax RRearWheel = new CANSparkMax(kRightRearPort, MotorType.kBrushless);

  DifferentialDrive roboDrive = new DifferentialDrive(LFrontWheel, RFrontWheel);

  AHRS gyro = new AHRS(SPI.Port.kMXP);

  // Autonomous Tracking
  DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
  DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  SimpleMotorFeedforward feedforward 
  = new SimpleMotorFeedforward(driveFeedforward.ks, driveFeedforward.kv, driveFeedforward.ka);

  PIDController leftDrivePID 
  = new PIDController(leftDrive.kP, leftDrive.kI, leftDrive.kD);

  PIDController rightDrivePID
  = new PIDController(rightDrive.kP, rightDrive.kI, rightDrive.kD);

  Pose2d pose = new Pose2d();

  public RevDrivetrain() {
    LRearWheel.follow(LFrontWheel);
    RRearWheel.follow(RFrontWheel);

    LFrontWheel.getEncoder().setPosition(0);
    RFrontWheel.getEncoder().setPosition(0);

    gyro.reset();
  }

  public void setOutputVolts(double leftVolts, double rightVolts) {
    LFrontWheel.setVoltage(leftVolts);
    RFrontWheel.setVoltage(rightVolts);
  }

  public void setOutputFeedforward(double leftVolts, double rightVolts) {
    LFrontWheel.setVoltage(feedforward.calculate(leftVolts));
    RFrontWheel.setVoltage(feedforward.calculate(rightVolts));
  }

  public DifferentialDrive getDifferentialDrive() {
    return roboDrive;
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(-gyro.getAngle());
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }
  
  public Pose2d getPose() {
    return pose;
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftDrivePID() {
    return leftDrivePID;
  }

  public PIDController getRightDrivePID() {
    return rightDrivePID;
  }
  
  public double getLeftDistanceMeters() {
    return LFrontWheel.getEncoder().getPosition() / 
    RFrontWheel.getEncoder().getCountsPerRevolution() * 2 * Math.PI * kWheelRadiusMeters;
  }

  public double getRightDistanceMeters() {
    return RFrontWheel.getEncoder().getPosition() / 
    RFrontWheel.getEncoder().getCountsPerRevolution() * 2 * Math.PI * kWheelRadiusMeters;
      
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      LFrontWheel.getEncoder().getVelocity() / kGearRatio * 2 * Math.PI * kWheelRadiusMeters / 60,
      RFrontWheel.getEncoder().getVelocity() / kGearRatio * 2 * Math.PI * kWheelRadiusMeters / 60
    );
  }

  public void driveStraight(double leftSpeed, double rightSpeed) {
    double leftVelocity = LFrontWheel.getEncoder().getVelocity();
    double rightVelocity = RFrontWheel.getEncoder().getVelocity();
    double avgVelocity = (leftVelocity + rightVelocity)/2;
    
    if(Math.abs(leftSpeed - rightSpeed) < 0.05) {
      roboDrive.tankDrive(
        leftDrivePID.calculate(leftVelocity, avgVelocity),
        rightDrivePID.calculate(rightVelocity, avgVelocity));
    }
    else {
      roboDrive.tankDrive(leftSpeed, rightSpeed);
    }
  }
  /**
  * Will be called periodically whenever the CommandScheduler runs.
  */
  @Override
  public void periodic() {
      pose = odometry.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
  }
}
