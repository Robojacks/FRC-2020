/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.climber;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Arm extends SubsystemBase {
  private WPI_TalonSRX leftArm = new WPI_TalonSRX(0);
  private WPI_TalonSRX rightArm = new WPI_TalonSRX(1);

  public enum armState {
    REACH, PULL, NEUTRAL
  }

  // Keeps track of what mode arm is in
  private armState state = armState.NEUTRAL;

  public void reach(){
    leftArm.set(-armReachSpeed);
    rightArm.set(armReachSpeed);

    state = armState.REACH;
  }

  public void pull(){
    leftArm.set(-armPullSpeed);
    rightArm.set(armPullSpeed);

    state = armState.PULL;
  }

  public void stop() {
    leftArm.set(0);
    rightArm.set(0);

    state = armState.NEUTRAL;
  }


  public void switchArm() {
    switch(state) {
      case PULL:
        reach();
      case REACH:
        pull();
      case NEUTRAL: 
        pull();
      default:
        stop();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}


