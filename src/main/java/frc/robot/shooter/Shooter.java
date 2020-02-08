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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  private WPI_TalonSRX leftLauncher = new WPI_TalonSRX(kLeftShooterWheelPort);
  private WPI_TalonSRX rightLauncher = new WPI_TalonSRX(kRightShooterWheelPort);
  private WPI_TalonSRX conveyor = new WPI_TalonSRX(kConveyorBelt);
  private Turret goalMover;
  
  // Conversion factor from minutes to milliseconds
  private double minToMS = 600;

  public Shooter(Turret turret) {

    // Makes turret instance the same as in RobotContainer
    goalMover = turret;

    leftLauncher.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);
    rightLauncher.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 0);

    leftLauncher.setSensorPhase(true);
    rightLauncher.setSensorPhase(false);

    leftLauncher.config_kP(0, shooterLeftPID.Kp);    
    leftLauncher.config_kI(0, shooterLeftPID.Ki);
    leftLauncher.config_kD(0, shooterLeftPID.Kd);
    leftLauncher.config_kF(0, 1023 * kTicksPerRev / leftLauncher.getSelectedSensorVelocity());

    rightLauncher.config_kP(0, shooterRightPID.Kp);
    rightLauncher.config_kI(0, shooterRightPID.Ki);
    rightLauncher.config_kD(0, shooterRightPID.Kd);
    rightLauncher.config_kF(0, 1023 * kTicksPerRev / rightLauncher.getSelectedSensorVelocity());
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
  public void setPoseRPM(double inRPM, double outRPM, double beltVolts){
    if (goalMover.getCollecting()) {
      setRPM(-inRPM, -beltVolts);

    } else {
      setRPM(outRPM, beltVolts);
    }
  }

  /**
   * Sets the velocity of the shooter in RPM using two velocity control loops. Also sets a 
   * voltage for the conveyor belt, given that it does not have to be as accurate. Must be
   * set negative for intake.
   * @param launchRPM Velocity of the shooter in rotations per minute
   * @param beltVolts The applied voltage to the conveyor belt, subject to minor fluctuations
  */ 
  private void setRPM(double launchRPM, double beltVolts) {
    leftLauncher.set(ControlMode.Velocity, launchRPM * kTicksPerRev / minToMS);
    rightLauncher.set(ControlMode.Velocity, -launchRPM * kTicksPerRev / minToMS);

    conveyor.setVoltage(beltVolts);
  }

  public double getLeftVelocity() {
    return leftLauncher.getSelectedSensorVelocity() / kTicksPerRev * minToMS;
  }

  public int getRightVelocity() {
    return rightLauncher.getSelectedSensorVelocity();
  }

  @Override
  public void periodic() {
  }
}
