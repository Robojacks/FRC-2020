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
  private WPI_TalonSRX leftArm = new WPI_TalonSRX(leftArmPort);
  private WPI_TalonSRX rightArm = new WPI_TalonSRX(rightArmPort);

  private boolean negation = false;

  public void move(double speed) {
    leftArm.set(speed);
    rightArm.set(speed);
  }

  public void moveOneAxis(double speed) {
    if (negation) {
      leftArm.set(-speed);
      rightArm.set(-speed);
    } else {
      leftArm.set(speed);
      rightArm.set(speed);
    }
  }

  public void switchMovement() {
    if (negation) {
      negation = false;
    } else {
      negation = true;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}


