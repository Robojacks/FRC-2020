/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Plucker extends SubsystemBase {
  private ChangePosition goalMover;
  private Shooter m_shooter;

  private WPI_TalonSRX plucker = new WPI_TalonSRX(kPluckerPort);

  /**
   * Creates a new Plucker.
   */
  public Plucker(ChangePosition changePosition, Shooter shooter) {
    goalMover = changePosition;
    m_shooter = shooter;
  }

  public void setSpeedLowGoal(double inVolts, double outVolts){
    if (goalMover.getCollecting()) {
      plucker.setVoltage(-inVolts);

    } else {
      plucker.setVoltage(outVolts);
    }
  }

  public void setSpeedHighGoal(double inVolts, double outVolts){
    if (goalMover.getCollecting()) {
      plucker.setVoltage(-inVolts);

    } else {
      Timer.delay(shooterRampUpTime);
      plucker.setVoltage(outVolts);
    }
  }

  public void toggleSpeedLowGoal(double inPluckerVolts, double outPluckerVolts) {
    if (m_shooter.isEngaged()) {
      setSpeedLowGoal(inPluckerVolts, outPluckerVolts);
    } else {
      setSpeedLowGoal(0, 0);
    }
  }

  public void toggleSpeedHighGoal(double inPluckerVolts, double outPluckerVolts) {
    if (m_shooter.isEngaged()) {
      setSpeedHighGoal(inPluckerVolts, outPluckerVolts);
    } else {
      setSpeedHighGoal(0, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
