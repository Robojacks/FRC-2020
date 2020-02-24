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
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.robot.Gains.*;

import static frc.robot.Constants.*;

/**
 * Add your docs here.
 */
public class RevDrivetrain extends SubsystemBase {

  private CANSparkMax LFrontWheel = new CANSparkMax(kLeftFrontPort, MotorType.kBrushless);
  private CANSparkMax RFrontWheel = new CANSparkMax(kRightFrontPort, MotorType.kBrushless);

  private CANSparkMax LRearWheel = new CANSparkMax(kLeftRearPort, MotorType.kBrushless);
  private CANSparkMax RRearWheel = new CANSparkMax(kRightRearPort, MotorType.kBrushless);

  private DifferentialDrive roboDrive = new DifferentialDrive(LFrontWheel, RFrontWheel);

  private AHRS gyro = new AHRS(SPI.Port.kMXP);

  private double gyroPosition = -gyro.getAngle();

  private PIDController gyroController 
    = new PIDController(0.1, 0, 0);

  // Autonomous Tracking
  private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
  private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(getHeading());

  private SimpleMotorFeedforward feedforward 
    = new SimpleMotorFeedforward(driveFeedforward.ks, driveFeedforward.kv, driveFeedforward.ka);

  private PIDController leftDrivePID 
    = new PIDController(leftDrive.kP, leftDrive.kI, leftDrive.kD);

  private PIDController rightDrivePID
    = new PIDController(rightDrive.kP, rightDrive.kI, rightDrive.kD);

  private Pose2d pose = new Pose2d();

  private SlewRateLimiter speedLimiter = new SlewRateLimiter(3);

  public RevDrivetrain() {
    LRearWheel.follow(LFrontWheel);
    RRearWheel.follow(RFrontWheel);

    LFrontWheel.getEncoder().setPosition(0);
    RFrontWheel.getEncoder().setPosition(0);

    gyro.reset();
  }

  
  public void tankDriveWithGyro(double leftSpeed, double rightSpeed) {

    if(Math.abs(leftSpeed - rightSpeed) < straightDriveDeadband) {
      double speedAverage = (leftSpeed + rightSpeed) / 2;

      roboDrive.arcadeDrive(
        speedAverage, 

        // Keeps PID output within joystick range
        MathUtil.clamp(
          // calculates how far to adjust based off of the previously recorded angle
          gyroController.calculate(getAngle(), gyroPosition), 
        
          // low output, high output 
          -1, 1
        ),
        // Squared inputs
        false
      );

    } else {
      roboDrive.tankDrive(leftSpeed, rightSpeed, false);
      gyroPosition = getAngle();
    }

  }
  

  public void arcadeDriveWithGyro(double speed, double rotation) {
    // Drive in arcade mode (one axis for linear speed, one for rotation)
    roboDrive.arcadeDrive(
      // Limits rate of change of percent speed
      /*speedLimiter.calculate(speed),*/
      speed, 

      // Constrains PID output
      MathUtil.clamp(
        // Using the fused heading as a measure, adds rotation percent times magnitude
        gyroController.calculate(getAngle(), getAngle() + rotationMagnitude * rotation),
        
        // Low output, high output
        -1, 1
      ),

      // Squared inputs, false for linearity
      false
    );
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

  public double getAngle() {
    return -gyro.getAngle();
  }

  public Rotation2d getHeading() {
    return Rotation2d.fromDegrees(getAngle());
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

  /**
  * Will be called periodically whenever the CommandScheduler runs.
  */
  @Override
  public void periodic() {
      pose = odometry.update(getHeading(), getLeftDistanceMeters(), getRightDistanceMeters());
  }
}
