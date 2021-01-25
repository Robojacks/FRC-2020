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

public class Conveyor extends SubsystemBase {
  private ChangePosition goalMover;

  private WPI_TalonSRX conveyor = new WPI_TalonSRX(kConveyorPort);

  private boolean engaged = false;

  /**
   * Creates a new Conveyor.
   */
  public Conveyor(ChangePosition changePosition) {
    goalMover = changePosition;
  }

  public void stop() {
    conveyor.setVoltage(0);
    engaged = false;
  }

  public void collect() {
    conveyor.setVoltage(inConveyorVolts);
    engaged = true;
  }

  public void shoot() {
    conveyor.setVoltage(outConveyorVolts);
    engaged = true;
  }
  
  /**
   * Sets a voltage based on whether the robot is in low goal shooting position or
   * intake position. 
   */
  public void setSpeed() {
    if (goalMover.isCollectingPose()) {
      collect();

    } else {
      shoot();
    }
  }

  /**
   * Relying on the the shooter state, toggles conveyor on and off 
   * with the specified voltage
   * @param beltVolts voltage applied to the conveyor when it is on
   */
  public void toggleSpeed() {
    if (engaged) {
      stop();

    } else {
      setSpeed();

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
