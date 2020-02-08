/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.shooter.Shooter;

import static frc.robot.Constants.*;
/**
 * Mainly for logging values that need adjustment
 */
public class Update {
  private Shooter m_shooter;

  // Starting positions
  private final Pose2d left = new Pose2d(-1, 0, Rotation2d.fromDegrees(0));
  private final Pose2d center = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private final Pose2d right = new Pose2d(1, 0, Rotation2d.fromDegrees(0));

  private static final SendableChooser choosePosition = new SendableChooser<>();

  public Update(Shooter shooter) {
    m_shooter = shooter;

    choosePosition.setDefaultOption("Center", center);
    choosePosition.addOption("Left", left);
    choosePosition.addOption("Right", right);
    SmartDashboard.putData("Starting Position", choosePosition);

    // Display PID values (angle)
    SmartDashboard.putNumber("P value(angle)", angleCorrection.Kp);
    SmartDashboard.putNumber("I value(angle)", angleCorrection.Ki);
    SmartDashboard.putNumber("D value(angle)", angleCorrection.Kd);

    // Display PID values (distance)
    SmartDashboard.putNumber("P value(distance)", distanceCorrection.Kp);
    SmartDashboard.putNumber("I value(distance)", distanceCorrection.Ki);
    SmartDashboard.putNumber("D value(distance)", distanceCorrection.Kd);

    // Display color bounds
    SmartDashboard.putNumber("Blue Lower Bound", Constants.blueLowerBound);
    SmartDashboard.putNumber("Blue Upper Bound", Constants.blueUpperBound);

    SmartDashboard.putNumber("Red Lower Bound", Constants.redLowerBound);
    SmartDashboard.putNumber("Red Upper Bound", Constants.redUpperBound);

    SmartDashboard.putNumber("Green Lower Bound", Constants.greenLowerBound);
    SmartDashboard.putNumber("Green Upper Bound", Constants.greenUpperBound);
    
    SmartDashboard.putNumber("Yellow Lower Bound", Constants.yellowLowerBound);
    SmartDashboard.putNumber("Yellow Upper Bound", Constants.yellowUpperBound);

    // Display left and right shooter velocities
    SmartDashboard.putNumber("Left Shooter Velocity (RPM)", m_shooter.getLeftVelocity());
    SmartDashboard.putNumber("Right Shooter Velocity (RPM)", m_shooter.getRightVelocity());
  }

  public static Pose2d getStartingPose() {
    final Pose2d position = (Pose2d) choosePosition.getSelected();
    return position;
  }

  public void periodic() {
    // Update left and right shooter velocities
    SmartDashboard.putNumber("Left Shooter Velocity (RPM)", m_shooter.getLeftVelocity());
    SmartDashboard.putNumber("Right Shooter Velocity (RPM)", m_shooter.getRightVelocity());
    
    // Change PID values for angle correction
    if (angleCorrection.Kp != SmartDashboard.getNumber("P value(angle)", angleCorrection.Kp))  {
      angleCorrection.Kp = SmartDashboard.getNumber("P value(angle)", angleCorrection.Kp);
    }

    if (angleCorrection.Ki != SmartDashboard.getNumber("I value(angle)", angleCorrection.Ki))  {
      angleCorrection.Ki = SmartDashboard.getNumber("I value(angle)", angleCorrection.Ki);
    }

    if (angleCorrection.Kd != SmartDashboard.getNumber("D value(angle)", angleCorrection.Kd))  {
      angleCorrection.Kd = SmartDashboard.getNumber("D value(angle)", angleCorrection.Kd);
    } 

    // Change PID values for distance correction
    if (distanceCorrection.Kp != SmartDashboard.getNumber("P value(distance)", distanceCorrection.Kp))  {
      distanceCorrection.Kp = SmartDashboard.getNumber("P value(distance)", distanceCorrection.Kp);
    }

    if (distanceCorrection.Ki != SmartDashboard.getNumber("I value(distance)", distanceCorrection.Ki))  {
      distanceCorrection.Ki = SmartDashboard.getNumber("I value(distance)", distanceCorrection.Ki);
    }

    if (distanceCorrection.Kd != SmartDashboard.getNumber("D value(distance)", distanceCorrection.Kd))  {
      distanceCorrection.Kd = SmartDashboard.getNumber("D value(distance)", distanceCorrection.Kd);
    } 

     // Change color bounds
    if (blueLowerBound != SmartDashboard.getNumber("Blue Lower Bound", Constants.blueLowerBound))  {
      blueLowerBound = SmartDashboard.getNumber("Blue Lower Bound", Constants.blueLowerBound);
    }

    if (blueUpperBound != SmartDashboard.getNumber("Blue Upper Bound", Constants.blueUpperBound))  {
      blueUpperBound = SmartDashboard.getNumber("Blue Upper Bound", Constants.blueUpperBound);
    }

    if (redLowerBound != SmartDashboard.getNumber("Red Lower Bound", Constants.redLowerBound))  {
      redLowerBound = SmartDashboard.getNumber("Red Lower Bound", Constants.redLowerBound);
    }

    if (redUpperBound != SmartDashboard.getNumber("Red Upper Bound", Constants.redUpperBound))  {
      redUpperBound = SmartDashboard.getNumber("Red Upper Bound", Constants.redUpperBound);
    }

    if (greenLowerBound != SmartDashboard.getNumber("Green Lower Bound", Constants.greenLowerBound))  {
      greenLowerBound = SmartDashboard.getNumber("Green Lower Bound", Constants.greenLowerBound);
    }

    if (greenUpperBound != SmartDashboard.getNumber("Green Upper Bound", Constants.greenUpperBound))  {
      greenUpperBound = SmartDashboard.getNumber("Green Upper Bound", Constants.greenUpperBound);
    }

    if (yellowLowerBound != SmartDashboard.getNumber("Yellow Lower Bound", Constants.greenLowerBound))  {
      yellowLowerBound = SmartDashboard.getNumber("Yellow Lower Bound", Constants.greenLowerBound);
    }

    if (yellowUpperBound != SmartDashboard.getNumber("Yellow Upper Bound", Constants.yellowUpperBound))  {
      yellowUpperBound = SmartDashboard.getNumber("Yellow Upper Bound", Constants.yellowUpperBound);
    }
  }
}
