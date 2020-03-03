/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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

  public void setSpeed(double inVolts, double outVolts){
    if (goalMover.getCollecting()) {
      plucker.setVoltage(inVolts);

    } else {
      plucker.setVoltage(-outVolts);
    }
  }

  public void toggleSpeed(double inPluckerVolts, double outPluckerVolts) {
    if (m_shooter.isEngaged()) {
      setSpeed(inPluckerVolts, outPluckerVolts);
    } else {
      setSpeed(0, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
