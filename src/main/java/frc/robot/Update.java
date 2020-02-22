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
import frc.robot.wheel.SenseColor;

import static frc.robot.Constants.*;
import static frc.robot.Gains.*;

/**
 * Mainly for logging values that need adjustment
 */
public class Update {
  private Shooter m_shooter;
  private SenseColor colorSense;

  // Starting positions
  private final Pose2d left = new Pose2d(-1, 0, Rotation2d.fromDegrees(0));
  private final Pose2d center = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private final Pose2d right = new Pose2d(1, 0, Rotation2d.fromDegrees(0));

  private static final SendableChooser choosePosition = new SendableChooser<Pose2d>();

  public Update(SenseColor colorSensing, Shooter shooter) {
    m_shooter = shooter;
    colorSense = colorSensing;

    choosePosition.setDefaultOption("Center", center);
    choosePosition.addOption("Left", left);
    choosePosition.addOption("Right", right);
    SmartDashboard.putData("Starting Position", choosePosition);

    // Display PID values (angle)
    SmartDashboard.putNumber("P value(angle)", angleCorrection.kP);
    SmartDashboard.putNumber("I value(angle)", angleCorrection.kI);
    SmartDashboard.putNumber("D value(angle)", angleCorrection.kD);

    // Display PID values (distance)
    SmartDashboard.putNumber("P value(distance)", distanceCorrection.kP);
    SmartDashboard.putNumber("I value(distance)", distanceCorrection.kI);
    SmartDashboard.putNumber("D value(distance)", distanceCorrection.kD);

    // Display color bounds
    SmartDashboard.putNumber("Blue Lower Bound", Constants.blueLowerBound);
    SmartDashboard.putNumber("Blue Upper Bound", Constants.blueUpperBound);

    SmartDashboard.putNumber("Red Lower Bound", Constants.redLowerBound);
    SmartDashboard.putNumber("Red Upper Bound", Constants.redUpperBound);

    SmartDashboard.putNumber("Green Lower Bound", Constants.greenLowerBound);
    SmartDashboard.putNumber("Green Upper Bound", Constants.greenUpperBound);
    
    SmartDashboard.putNumber("Yellow Lower Bound", Constants.yellowLowerBound);
    SmartDashboard.putNumber("Yellow Upper Bound", Constants.yellowUpperBound);

    // Put up color sense data
    SmartDashboard.putNumber("Raw Color Value", colorSense.getRawColor());
    SmartDashboard.putNumber("Proximity", colorSense.getProximity());
    SmartDashboard.putString("Detected Color", colorSense.getColorString());

    // Display left and right shooter velocities
    SmartDashboard.putNumber("Shooter RPM", m_shooter.getVelocity());
  }

  public static Pose2d getStartingPose() {
    final Pose2d position = (Pose2d) choosePosition.getSelected();
    return position;
  }

  public void periodic() {
    // Update left and right shooter velocities
    SmartDashboard.putNumber("Shooter RPM", m_shooter.getVelocity());

    // Update color sense data
    SmartDashboard.putNumber("Raw Color Value", colorSense.getRawColor());
    SmartDashboard.putNumber("Proximity", colorSense.getProximity());
    SmartDashboard.putString("Detected Color", colorSense.getColorString());
    
    // Change PID values for angle correction
    if (angleCorrection.kP != SmartDashboard.getNumber("P value(angle)", angleCorrection.kP))  {
      angleCorrection.kP = SmartDashboard.getNumber("P value(angle)", angleCorrection.kP);
    }

    if (angleCorrection.kI != SmartDashboard.getNumber("I value(angle)", angleCorrection.kI))  {
      angleCorrection.kI = SmartDashboard.getNumber("I value(angle)", angleCorrection.kI);
    }

    if (angleCorrection.kD != SmartDashboard.getNumber("D value(angle)", angleCorrection.kD))  {
      angleCorrection.kD = SmartDashboard.getNumber("D value(angle)", angleCorrection.kD);
    } 

    // Change PID values for distance correction
    if (distanceCorrection.kP != SmartDashboard.getNumber("P value(distance)", distanceCorrection.kP))  {
      distanceCorrection.kP = SmartDashboard.getNumber("P value(distance)", distanceCorrection.kP);
    }

    if (distanceCorrection.kI != SmartDashboard.getNumber("I value(distance)", distanceCorrection.kI))  {
      distanceCorrection.kI = SmartDashboard.getNumber("I value(distance)", distanceCorrection.kI);
    }

    if (distanceCorrection.kD != SmartDashboard.getNumber("D value(distance)", distanceCorrection.kD))  {
      distanceCorrection.kD = SmartDashboard.getNumber("D value(distance)", distanceCorrection.kD);
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
