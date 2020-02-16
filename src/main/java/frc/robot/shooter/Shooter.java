/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  private ChangePosition goalMover;

  private WPI_TalonSRX leftLauncher = new WPI_TalonSRX(kLeftShooterWheelPort);
  private WPI_TalonSRX rightLauncher = new WPI_TalonSRX(kRightShooterWheelPort);
  
  private PIDController leftControl 
    = new PIDController(shooterLeftPID.Kp, shooterLeftPID.Ki, shooterLeftPID.Kd);

  private PIDController rightControl 
    = new PIDController(shooterRightPID.Kp, shooterRightPID.Ki, shooterRightPID.Kd);

  private SimpleMotorFeedforward feedforward 
    = new SimpleMotorFeedforward(shooterFeedforward.ks, shooterFeedforward.kv);

  // Create toggle for shooting
  private boolean engaged = false;

  // Conversion factor from minutes to milliseconds
  private double minToMS = 600;

  public Shooter(ChangePosition changePosition) {
    // Makes changePosition instance the same as in RobotContainer
    goalMover = changePosition;

    leftLauncher.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    rightLauncher.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    leftLauncher.setSensorPhase(true);
    rightLauncher.setSensorPhase(false);

    // WPILib PID Stuff
    leftControl.setTolerance(shooterLeftPID.tolerance);
    rightControl.setTolerance(shooterRightPID.tolerance);

    // Talon PID Stuff
    leftLauncher.config_kP(0, shooterLeftPID.Kp);    
    leftLauncher.config_kI(0, shooterLeftPID.Ki);
    leftLauncher.config_kD(0, shooterLeftPID.Kd);
    leftLauncher.config_kF(0, shooterLeftPID.Kf);

    rightLauncher.config_kP(0, shooterRightPID.Kp);
    rightLauncher.config_kI(0, shooterRightPID.Ki);
    rightLauncher.config_kD(0, shooterRightPID.Kd);
    rightLauncher.config_kF(0, shooterRightPID.Kf);
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
      leftLauncher.setVoltage(-inVolts);
      rightLauncher.setVoltage(-inVolts);

    } else {
      leftLauncher.setVoltage(outVolts);
      rightLauncher.setVoltage(outVolts);
    }
  }

  /**
   * Toggles shooter on and off with the specified voltage
   * @param inVolts voltage applied with intake
   * @param outVolts voltage applied with shooting
   */
  public void toggleSpeedVolts(double inVolts, double outVolts) {
    if (engaged) {
      setSpeedVolts(0, 0);
      engaged = false;

    } else {
      setSpeedVolts(inVolts, outVolts);
      engaged = true;
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
  public void setSpeedTalon(double inRPM, double outRPM){
    if (goalMover.getCollecting()) {
      leftLauncher.set(ControlMode.Velocity, -inRPM * kTicksPerRev / minToMS);
      rightLauncher.set(ControlMode.Velocity, -inRPM * kTicksPerRev / minToMS);

    } else {
      leftLauncher.set(ControlMode.Velocity, outRPM * kTicksPerRev / minToMS);
      rightLauncher.set(ControlMode.Velocity, outRPM * kTicksPerRev / minToMS);
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
    leftLauncher.setVoltage(
      MathUtil.clamp(feedforward.calculate(launchRPM) // feedforward
      + leftControl.calculate(getRawLeftVelocity(), launchRPM), // PID correction
      -12, 12)); // min volts, max volts

    rightLauncher.setVoltage(
      MathUtil.clamp(feedforward.calculate(-launchRPM) // feedforward
      + rightControl.calculate(getRawRightVelocity(), -launchRPM), // PID correction
      -12, 12)); // min volts, max volts
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

  public boolean isEngaged() {
    return engaged;
  }

  public double getRawLeftVelocity() {
    return leftLauncher.getSelectedSensorVelocity();
  }

  public double getRawRightVelocity() {
    return rightLauncher.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
  }
}
