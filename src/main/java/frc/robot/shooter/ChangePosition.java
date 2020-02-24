/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shooter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class ChangePosition extends SubsystemBase {
  private Shooter m_shooter;

  private Compressor airow = new Compressor(20);

  private Solenoid leftPiston = new Solenoid(compressorModule, leftPistonPort);
  private Solenoid rightPiston = new Solenoid(compressorModule, rightPistonPort);

  private boolean collecting = false;

  public ChangePosition(Shooter shooter) {
    m_shooter = shooter;

    airow.start();
  }

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

    m_shooter.stop(); // For safety, stops shooter when changing position

    collecting = true;
    System.out.println("Collecting Pose Set");
  }

  public void shootPose(){
    leftPiston.set(false);
    rightPiston.set(false);

    m_shooter.stop(); // For safety, stops shooter when changing position

    collecting = false;
    System.out.println("Shooting Pose Set");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
