/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.drive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 
import static frc.robot.Constants.*;
/**
 * Add your docs here.
 */
public class Gears extends SubsystemBase {
  Solenoid left = new Solenoid(compressorModule, SolGearLPort);
  Solenoid right = new Solenoid(compressorModule, SolGearRPort);

  public enum gearState {
    FAST, SLOW;
  }

  // Keeps track of how high the shooter is
  private gearState state = gearState.SLOW;

  public void FAST(){
    left.set(true);
    right.set(true);
    state = gearState.FAST;
  }

  public void SLOW(){
    left.set(false);
    right.set(false);
    state = gearState.SLOW;
  }

  public void switchGear() {
    switch(state) {
      case FAST:
        FAST();
      case SLOW:
        SLOW();
      default:
        SLOW();
    }
  }
}
