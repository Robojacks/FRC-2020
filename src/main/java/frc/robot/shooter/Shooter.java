/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shooter;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.robot.Gains.shooterPID;
import static frc.robot.Gains.shooterFeedforward;

import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  private ChangePosition goalMover;

  private CANSparkMax launcher = new CANSparkMax(kShooterPort, MotorType.kBrushless);

  private CANEncoder launcherEncoder = launcher.getEncoder();
  
  private CANPIDController launcherController = new CANPIDController(launcher);

  // Create toggle for shooting
  private boolean engaged = false;

  public Shooter(ChangePosition changePosition) {
    // Makes changePosition instance the same as in RobotContainer
    goalMover = changePosition;

    // Spark PID Stuff
    launcherController.setP(shooterPID.kP);
    launcherController.setI(shooterPID.kI);
    launcherController.setD(shooterPID.kD); 
    launcherController.setFF(shooterPID.kF);

    launcherController.setOutputRange(-1000, 1000);
    //launcherEncoder.setVelocityConversionFactor(factor)
  }

  public void stop() {
    launcher.setVoltage(0);
    launcherController.setReference(0, ControlType.kVoltage);

    engaged = false;
  }

  /**
   * Sets a voltage based on whether the robot is in shooting position or
   * intake position. 
   * @param inVolts The voltage sent to the shooter while in the intake position
   * @param outVolts The voltage sent to the shooter while in the shooting position
   */
  public void setSpeedVolts(double inVolts, double outVolts){
    if (goalMover.getCollecting()) {
      // if in intake position, intake
      launcher.setVoltage(-inVolts);
      engaged = true;

    } else {
      launcher.setVoltage(outVolts);
      engaged = true;

    }
  }

  /**
   * Toggles shooter on and off with the specified voltage
   * @param inVolts voltage applied with intake
   * @param outVolts voltage applied with shooting
   */
  public void toggleSpeedVolts(double inVolts, double outVolts) {
    if (engaged) {
      stop();

    } else {
      setSpeedVolts(inVolts, outVolts);
    }
  }

  /**
   * Sets RPM based on whether the robot is in shooting position or 
   * intake position. 
   * @param inRPM The velocity in RPM the shooter goes to while in the intake position
   * @param outRPM The velocity in RPM the shooter goes to while in the shooting position
   */
  public void setSpeedSpark(double inRPM, double outRPM){
    if (goalMover.getCollecting()) {
      launcherController.setReference(-inRPM, ControlType.kVelocity);
      engaged = true;

    } else {
      launcherController.setReference(outRPM, ControlType.kVelocity);
      engaged = true;

    }
  }

  public void toggleSpeedSpark(double inRPM, double outRPM) {
    if (engaged) {
      stop();

    } else {
      setSpeedSpark(inRPM, outRPM);

    }
  }

  /**
   * Using a ballistics equation and input distance, converts the output of meters per second
   * to RPM which can be output by the shooter
   * @param distance distance from the target
   * @return Rotations Per Minute (RPM) required to shoot a ball into the goal
   */
  private double calculateRPM(double distance) {
    // Get the velocity needed to shoot the ball
    double top = Math.sqrt(-4.9 * Math.pow(distance, 2)); 
    double bottom = Math.sqrt(Math.pow(Math.cos(Math.toRadians(shooterAngle)), 2) *
     (highGoalHeight - shooterHeight - Math.tan(shooterAngle) * distance));

    double metersPerSecond = top/bottom;

    double radiansPerSecond = metersPerSecond / kShooterWheelRadiusMeters;

    return Units.radiansPerSecondToRotationsPerMinute(radiansPerSecond);
  }

  /**
   * Sets RPM based on whether the robot is in shooting position or intake position. 
   * If it is in shooting position, calculates the RPM needed for the shooter to 
   * hit the target.
   * @param inRPM the velocity in RPM the shooter goes to while in the intake position
   * @param distance the distance in meters from the target
   */
  public void setRelativeSpeedSpark(double inRPM, double distance){
    if (goalMover.getCollecting()) {
      launcherController.setReference(-inRPM, ControlType.kVelocity);
      engaged = true;

    } else {
      launcherController.setReference(calculateRPM(distance), ControlType.kVelocity);
      engaged = true;

    }
  }

  public void toggleRelativeSpeedSpark(double inRPM, double distance) {
    if (engaged) {
      stop();

    } else {
      setSpeedSpark(inRPM, distance);

    }
  }

  public boolean isEngaged() {
    return engaged;
  }

  /**
   * Gets the shooter velocity in RPM
   */
  public double getVelocity() {
    return launcherEncoder.getVelocity();
  }

  @Override
  public void periodic() {
  }
}
