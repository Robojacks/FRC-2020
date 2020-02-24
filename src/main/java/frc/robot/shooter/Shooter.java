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

  private PIDController shootControl 
    = new PIDController(shooterPID.kP, shooterPID.kI, shooterPID.kD);

  private SimpleMotorFeedforward feedforward 
    = new SimpleMotorFeedforward(shooterFeedforward.ks, shooterFeedforward.kv);

  // Create toggle for shooting
  private boolean engaged = false;

  // Conversion factor from minutes to milliseconds
  private double minToMS = 600;

  public Shooter(ChangePosition changePosition) {
    // Makes changePosition instance the same as in RobotContainer
    goalMover = changePosition;

    launcherEncoder.setInverted(false);

    // WPILib PID Stuff
    shootControl.setTolerance(shooterPID.tolerance);

    // Spark PID Stuff
    launcherController.setP(shooterPID.kP);
    launcherController.setI(shooterPID.kI);
    launcherController.setD(shooterPID.kD); 
    launcherController.setFF(shooterPID.kF);

    launcherController.setOutputRange(-3000, 3000);
    //launcherEncoder.setVelocityConversionFactor(factor)
  }

  public void stop() {
    launcher.setVoltage(0);

    shootControl.reset();
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
   * @param beltVolts The voltage sent to the conveyor belt, changing direction
   * depending on whether the shooter is in the intake or shooting position
   */
  public void setSpeedSpark(double inRPM, double outRPM){
    if (goalMover.getCollecting()) {
      launcherController.setReference(-inRPM, ControlType.kVelocity);

    } else {
      launcherController.setReference(outRPM, ControlType.kVelocity);

    }
  }

  public void toggleSpeedSpark(double inRPM, double outRPM) {
    if (engaged) {
      stop();
      engaged = false;

    } else {
      setSpeedSpark(inRPM, outRPM);
      engaged = true;

    }
  }

  /**
   * Sets the velocity of the shooter in RPM using two velocity control loops through the roborio.
   * Also sets a voltage for the conveyor belt, given that it does not have to be as accurate. Must 
   * be set negative for intake.
   * @param launchRPM Velocity of the shooter in rotations per minute
   * @param beltVolts The applied voltage to the conveyor belt, subject to minor fluctuations
  */ 
  private void setRPMWPI(double launchRPM) {
    launcher.setVoltage(
      MathUtil.clamp(
        // feedforward
        feedforward.calculate(launchRPM) +
        // PID correction
        shootControl.calculate(getVelocity(), launchRPM), 
        // min volts, max volts
        -12, 12
      )
    ); 
  }

  /**
   * Sets RPM based on whether the robot is in shooting position or 
   * intake position. 
   * @param inRPM The velocity in RPM the shooter goes to while in the intake position
   * @param outRPM The velocity in RPM the shooter goes to while in the shooting position
   */
  public void setSpeedWPI(double inRPM, double outRPM){
    if (goalMover.getCollecting()) {
      setRPMWPI(-inRPM);

    } else {
      setRPMWPI(outRPM);
    }
  }

  /**
   * Toggles shooter on and off with the specified RPM
   * @param inVolts voltage applied with intake
   * @param outVolts voltage applied with shooting
   */
  public void toggleSpeedWPI(double inRPM, double outRPM) {
    if (engaged) {
      stop();
      engaged = false;

    } else {
      setSpeedWPI(inRPM, outRPM);
      engaged = true;

    }
  }

  public boolean isEngaged() {
    return engaged;
  }

  /**
   * Gets the shooter velocity in RPM
   */
  public double getVelocity() {
    return launcherEncoder.getVelocity() * minToMS / kTicksPerRev;
  }

  @Override
  public void periodic() {
  }
}
