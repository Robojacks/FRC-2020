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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  private ChangePosition goalMover;

  private WPI_TalonSRX leftLauncher = new WPI_TalonSRX(kLeftShooterWheelPort);
  private WPI_TalonSRX rightLauncher = new WPI_TalonSRX(kRightShooterWheelPort);
  private WPI_TalonSRX conveyor = new WPI_TalonSRX(kConveyorBelt);
  
  private PIDController leftControl = new PIDController(0, 0, 0);
  private PIDController rightControl = new PIDController(0, 0, 0);

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
   * @param beltVolts The voltage sent to the conveyor belt, changing direction
   * depending on whether the shooter is in the intake or shooting position
   */
  public void setPoseVolts(double inVolts, double outVolts, double beltVolts){
    if (goalMover.getCollecting()) {
      collectVolts(inVolts, beltVolts);

    } else {
      shootVolts(outVolts, beltVolts);
    }
  }

  private void collectVolts(double launchVolts, double beltVolts){
    leftLauncher.setVoltage(-launchVolts);
    rightLauncher.setVoltage(launchVolts);
    conveyor.setVoltage(-beltVolts);

    System.out.println("Collecting " + launchVolts);
  }

  private void shootVolts(double launchVolts, double beltVolts){
    leftLauncher.setVoltage(launchVolts);
    rightLauncher.setVoltage(-launchVolts);
    conveyor.setVoltage(beltVolts);

    System.out.println("Shooting " + launchVolts);
  }

  /**
   * Sets RPM based on whether the robot is in shooting position or 
   * intake position. 
   * @param inRPM The velocity in RPM the shooter goes to while in the intake position
   * @param outRPM The velocity in RPM the shooter goes to while in the shooting position
   * @param beltVolts The voltage sent to the conveyor belt, changing direction
   * depending on whether the shooter is in the intake or shooting position
   */
  public void setPoseRPMTalon(double inRPM, double outRPM, double beltVolts){
    if (goalMover.getCollecting()) {
      setRPMTalon(-inRPM, -beltVolts);

    } else {
      setRPMTalon(outRPM, beltVolts);
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
  public void setPoseRPMWPI(double inRPM, double outRPM, double beltVolts){
    if (goalMover.getCollecting()) {
      setRPMWPI(-inRPM, -beltVolts);

    } else {
      setRPMWPI(outRPM, beltVolts);
    }
  }

  /**
   * Sets the velocity of the shooter in RPM using two velocity control loops directly on the 
   * talon. Also sets a voltage for the conveyor belt, given that it does not have to be 
   * as accurate. Must be set negative for intake.
   * @param launchRPM Velocity of the shooter in rotations per minute
   * @param beltVolts The applied voltage to the conveyor belt, subject to minor fluctuations
  */ 
  private void setRPMTalon(double launchRPM, double beltVolts) {
    leftLauncher.set(ControlMode.Velocity, launchRPM * kTicksPerRev / minToMS);
    rightLauncher.set(ControlMode.Velocity, -launchRPM * kTicksPerRev / minToMS);

    conveyor.setVoltage(beltVolts);
  }

  /**
   * Sets the velocity of the shooter in RPM using two velocity control loops through the roborio.
   * Also sets a voltage for the conveyor belt, given that it does not have to be as accurate. Must 
   * be set negative for intake.
   * @param launchRPM Velocity of the shooter in rotations per minute
   * @param beltVolts The applied voltage to the conveyor belt, subject to minor fluctuations
  */ 
  private void setRPMWPI(double launchRPM, double beltVolts) {
    leftLauncher.set(
      MathUtil.clamp(leftControl.calculate(getLeftVelocity(), launchRPM), -.5, .5));

    rightLauncher.set(
      MathUtil.clamp(rightControl.calculate(getRightVelocity(), launchRPM), -.5, .5));

    conveyor.setVoltage(beltVolts);
  }

  /**
   * Sets the velocity of the shooter in RPM using two velocity control loops directly on the 
   * talon. Also sets a voltage for the conveyor belt, given that it does not have to be 
   * as accurate. Must be set negative for intake.
   * @param launchRPM Velocity of the shooter in rotations per minute
   * @param beltVolts The applied voltage to the conveyor belt, subject to minor fluctuations
  */ 
  private void setRPM(double launchRPM, double beltVolts, double error) {
    leftLauncher.setVoltage(launchRPM * kTicksPerRev / minToMS);
    rightLauncher.set(ControlMode.Velocity, -launchRPM * kTicksPerRev / minToMS);

    conveyor.setVoltage(beltVolts);
  }

  public double getLeftVelocity() {
    return leftLauncher.getSelectedSensorVelocity() / kTicksPerRev * minToMS;
  }

  public double getRightVelocity() {
    return rightLauncher.getSelectedSensorVelocity() / kTicksPerRev * minToMS;
  }

  @Override
  public void periodic() {
  }
}
