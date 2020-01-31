/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // Port Numbers
	public static final int kLeftFrontPort = 0;
	public static final int kRightFrontPort = 1;
	public static final int kLeftRearPort = 2;
  public static final int kRightRearPort = 4;

  public static final int kXboxPort = 0;

  public static final int kSpinnerPort = 8; 

  public static final int compressorModule = 20;

  public static final int leftPistonPort = 0;
	public static final int rightPistonPort = 1; 

  public static final int armPort = 7;

  public static final int kLeftShooterWheelPort = 5;
  public static final int kRightShooterWheelPort = 6;

  public static final int kConveyorBelt = 9;

    
  // Motor Phase
  public static final boolean kLeftInverted = false;
  public static final boolean kRightInverted = true;

  // Encoder Sensor Phase
  public static final boolean kLeftEncoderPhase = false;
  public static final boolean kRightEncoderPhase = false;

  // Robot Measurements
  public static final int kTicksPerRev = 4096;
  public static final double ktrackWidthMeters = Units.inchesToMeters(10);
  public static final double kGearRatio = 7.29;
  public static final double kWheelRadiusMeters = Units.inchesToMeters(3.0);
	public static final double MaxSafeVelocityMeters = Units.feetToMeters(2);
	public static final double MaxSafeAccelerationMeters = Units.feetToMeters(2);
	public static final double targetToCameraHeight = 5;
  public static final double cameraAngle = 0;
  
  // Color Value Bounds
	public static final double blueLowerBound = 106;
  public static final double blueUpperBound = 115;

  public static final double redLowerBound = 65;
  public static final double redUpperBound = 81;

  public static final double greenLowerBound = 46;
  public static final double greenUpperBound = 60;

  public static final double yellowLowerBound = 91;
  public static final double yellowUpperBound = 98;
  
  // Constant Speeds
  public static final double armPercentSpeed = 0.4;
  public static final double shooterVolts = 5; 
  public static final double shooterRPM = 30;
  public static final double shooterError = 20;

  public static final double armReachSpeed = 0.1;
  public static final double armPullSpeed = -0.1;
  
  // Field Measurements
  public static final double cameraToTargetHeight = 0;

  public static final double shooterDistanceFromTargetMeters = 5;
  
  // PID Constants
  public static class leftDrive {
    public static double Kp = 0.1;
    public static double Ki = 0;
    public static double Kd = 0;
  }

  public static class rightDrive {
    public static double Kp = 0.1;
    public static double Ki = 0;
    public static double Kd = 0;
  }

  public static class shooterLeftPID {
    public static double Kp = 0.1;
    public static double Ki = 0;
    public static double Kd = 0;
  }

  public static class shooterRightPID {
    public static double Kp = 0.1;
    public static double Ki = 0;
    public static double Kd = 0;
  }

  public static class distanceCorrection {
    public static double Kp = 0.1;
    public static double Ki = 0;
    public static double Kd = 0;
  }

  public static class angleCorrection {
    public static double Kp = 0.1;
    public static double Ki = 0;
    public static double Kd = 0;

  public static class spinnerWheel{
    public static double Kp = 0.1;
    public static double Ki = 0;
    public static double Kd = 0;
  }
  }

  // Ramsete controller constants
  public static class Ramsete {
    public static final double kb = 2;
    public static final double kzeta = 0.7;
  }

  // Feed Forward Constants
  public static class driveFeedforward {
    public static final double ks = 0;
    public static final double kv = 0;
    public static final double ka = 0;
  }
  
}