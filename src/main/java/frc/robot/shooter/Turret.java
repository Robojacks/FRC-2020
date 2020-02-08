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
  Solenoid piston = new Solenoid(leftPistonPort);

  public enum shooterState {
    COLLECTING, SHOOTING;
  }

  // Keeps track of how high the shooter is
  private shooterState state = shooterState.COLLECTING;

  private boolean collecting = false;

  public shooterState getState() {
    return state;
  }

  public boolean getCollecting() {
    return collecting;
  }

  /**
   * Switch from a shooting position to a collecting position and vice versa.
   */ 
  public void swapHeight(){
    switch(state){
      case COLLECTING:
        shootPose();
      case SHOOTING:
        collectPose();
    }
    
  }

  public void collectPose() {
    piston.set(false);
    state = shooterState.COLLECTING;
  }

  public void shootPose(){
    piston.set(true);
    state = shooterState.SHOOTING;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
