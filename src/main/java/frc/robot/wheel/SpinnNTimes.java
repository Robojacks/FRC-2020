

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.wheel;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.wheel.SenseColor.Colour;

public class SpinnNTimes extends CommandBase {
  /**
   * Creates a new SpinnNTimes.
   *
   */
  int rotation = 20;
  private final SenseColor colorSense = new SenseColor();
  private final Colour colorSensed = colorSense.getColour();
  private final Spinner spinner = new Spinner();

  public SpinnNTimes() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (rotation>0){
      spinner.nextColor(colorSensed);
      spinner.toSelectedColor2(colorSensed);
    }
    rotation=rotation-1;

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
