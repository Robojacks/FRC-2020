/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shooter;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Turret extends SubsystemBase {
  Solenoid leftPiston = new Solenoid(leftPistonPort);
  Solenoid rightPiston = new Solenoid(rightPistonPort);
  private boolean collecting = false;

  public boolean getCollecting() {
    return collecting;
  }

  /**
   * Switch from a shooting position to a collecting position and vice versa.
   */ 
  public void swapHeight(){
    if (collecting) {
      shootPose();
    } else {
      collectPose();
    } 
  }

  public void collectPose() {
    leftPiston.set(true);
    rightPiston.set(true);

    collecting = true;
    System.out.println("Collecting Pose Set");
  }

  public void shootPose(){
    leftPiston.set(false);
    rightPiston.set(false);

    collecting = false;
    System.out.println("Shooting Pose Set");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
