/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.turret;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Turret extends SubsystemBase {
  Solenoid piston = new Solenoid(leftPistonPort);

  public enum shooterState {
    COLLECTING, SHOOTING;
  }

  // Keeps track of how high the shooter is
  private shooterState state = shooterState.SHOOTING;

  public shooterState getState() {
    return state;
  }

  /**
   * Switch from a shooting position to a collecting position and vice versa.
   */ 
  public void swapHeight(){
    switch(state){
      case COLLECTING:
        shootGoal();
      
      case SHOOTING:
        collectGoal();
      
      default:
        collectGoal();
    }
    
  }

  public void collectGoal() {
    state = shooterState.COLLECTING;

    piston.set(false);
  }

  public void shootGoal() {
    state = shooterState.SHOOTING;
    
    piston.set(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
