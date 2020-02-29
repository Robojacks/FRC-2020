/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.drive.RevDrivetrain;

import static frc.robot.Constants.*;
import static frc.robot.Gains.*;

public class FollowTarget extends CommandBase {
  private Limelight vision;
  private RevDrivetrain drive;

  private PIDController angleCorrector 
    = new PIDController(angleCorrection.kP, angleCorrection.kI, angleCorrection.kD);
  
  /**
   * Creates a new FollowTarget.
   */
  public FollowTarget(Limelight limelight, RevDrivetrain rDrive) {
    vision = limelight;
    drive = rDrive;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision, drive);

    angleCorrector.setTolerance(0.2);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angleCorrector.setSetpoint(0);
    vision.lightOn();
    vision.visionMode();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Passing aim PID output to the drive
    drive.getDifferentialDrive().arcadeDrive(
      0,
      // Angle Correction
      MathUtil.clamp(
        // Calculate what to do based off measurement
        angleCorrector.calculate(-vision.getXError()),
        // Min, Max output
        -0.5, 0.5));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleCorrector.reset();
  }

}
