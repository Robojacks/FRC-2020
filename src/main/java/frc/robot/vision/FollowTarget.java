/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.vision;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drive.RevDrivetrain;

import static frc.robot.Constants.*;

public class FollowTarget extends CommandBase {
  private Limelight limelight = new Limelight();
  private RevDrivetrain rdrive = new RevDrivetrain();

  private PIDController distanceCorrector 
    = new PIDController(distanceCorrection.Kp, distanceCorrection.Ki, distanceCorrection.Kd);

  private PIDController angleCorrector 
    = new PIDController(angleCorrection.Kp, angleCorrection.Ki, angleCorrection.Kd);
  
  /**
   * Creates a new FollowTarget.
   */
  public FollowTarget() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(limelight, rdrive);

    distanceCorrector.setSetpoint(shooterDistanceFromTargetMeters);
    angleCorrector.setSetpoint(0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    rdrive.getDifferentialDrive().arcadeDrive(
    distanceCorrector.calculate(limelight.getTargetDistanceMeasured(cameraToBallTargetHeight, cameraAngle)), 
    angleCorrector.calculate(limelight.getXError()));
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
