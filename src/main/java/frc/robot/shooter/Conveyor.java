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

public class Conveyor extends SubsystemBase {
  private ChangePosition goalMover;
  private Shooter m_shooter;

  private WPI_TalonSRX conveyor = new WPI_TalonSRX(kConveyorPort);
  private WPI_TalonSRX feeder = new WPI_TalonSRX(kFeederPort);

  /**
   * Creates a new Conveyor.
   */
  public Conveyor(ChangePosition changePosition, Shooter shooter) {
    goalMover = changePosition;
    m_shooter = shooter;
  }

/**
   * Sets a voltage based on whether the robot is in low goal shooting position or
   * intake position. 
   * @param beltVolts The voltage sent to the conveyor belt, changing direction
   * depending on whether the shooter is in the intake or shooting position
   */
  public void setSpeedLowGoal(double beltVolts, double feederVolts){
    if (goalMover.getCollecting()) {
      conveyor.setVoltage(beltVolts);
      feeder.setVoltage(feederVolts);

    } else {
      conveyor.setVoltage(-beltVolts);
      feeder.setVoltage(-feederVolts);
    }
  }

  /**
   * Sets a voltage based on whether the robot is in high goal shooting position or
   * intake position. 
   * @param beltVolts The voltage sent to the conveyor belt, changing direction
   * depending on whether the shooter is in the intake or shooting position
   */
  public void setSpeedHighGoal(double beltVolts, double feederVolts){
    if (goalMover.getCollecting()) {
      conveyor.setVoltage(beltVolts);
      feeder.setVoltage(feederVolts);

    } else {
      // Delays start up time when in shooting position
      Timer.delay(shooterRampUpTime);
      conveyor.setVoltage(-beltVolts);
      feeder.setVoltage(-feederVolts);
    }
  }

  /**
   * Relying on the the shooter state, toggles conveyor on and off 
   * with the specified voltage
   * @param beltVolts voltage applied to the conveyor when it is on
   */
  public void toggleSpeedLowGoal(double beltVolts, double feederVolts) {
    if (m_shooter.isEngaged()) {
      setSpeedLowGoal(beltVolts, feederVolts);
    } else {
      setSpeedLowGoal(0, 0);
    }
  }

  /**
   * Relying on the the shooter state, toggles conveyor on and off 
   * with the specified voltage
   * @param beltVolts voltage applied to the conveyor when it is on
   */
  public void toggleSpeedHighGoal(double beltVolts, double feederVolts) {
    if (m_shooter.isEngaged()) {
      setSpeedHighGoal(beltVolts, feederVolts);
    } else {
      setSpeedHighGoal(0, 0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
