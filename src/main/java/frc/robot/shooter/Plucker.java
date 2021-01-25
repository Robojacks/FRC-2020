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

  private WPI_TalonSRX plucker = new WPI_TalonSRX(kPluckerPort);

  private boolean engaged = false;

  /**
   * Creates a new Plucker.
   */
  public Plucker(ChangePosition changePosition) {
    goalMover = changePosition;
  }

  public void stop() {
    plucker.setVoltage(0);
    engaged = false;
  }

  public void collect() {
    plucker.setVoltage(inPluckerVolts);
    engaged = true;
  }
  
  public void shoot() {
    plucker.setVoltage(outPluckerVolts);
    engaged = true;
  }

  public void setSpeed(){
    if (goalMover.isCollectingPose()) {
      collect();

    } else {
      shoot();
    }
  }

  public void toggleSpeed() {
    if (engaged) {
      stop();

    } else {
      setSpeed();
    }
  }

  public boolean getEngaged() {
    return engaged;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
