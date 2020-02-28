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
import frc.robot.wheel.Spinner;

import static frc.robot.Constants.*;
import static frc.robot.Gains.*;

/**
 * Mainly for logging values that need adjustment
 */
public class Update {
  private Shooter m_shooter;
  private SenseColor colorSense;
  private Spinner m_spinner;
  // Starting positions
  private final Pose2d left = new Pose2d(-1, 0, Rotation2d.fromDegrees(0));
  private final Pose2d center = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
  private final Pose2d right = new Pose2d(1, 0, Rotation2d.fromDegrees(0));

  private static final SendableChooser choosePosition = new SendableChooser<Pose2d>();



  public Update(SenseColor colorSensing, Shooter shooter, Spinner spinner) {
    m_shooter = shooter;
    colorSense = colorSensing;
    m_spinner = spinner;

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

   
    // Put up color sense data
    SmartDashboard.putNumber("Raw Color Value", colorSense.getRawColor());
    SmartDashboard.putNumber("Proximity", colorSense.getProximity());
    SmartDashboard.putString("Detected Color", colorSense.getColorString());

    // Display left and right shooter velocities
    SmartDashboard.putNumber("Shooter RPM", m_shooter.getVelocity());
    // Display color related value
    SmartDashboard.putString("colorFRC", colorSense.getColorString());
    SmartDashboard.putNumber("confidence", colorSense.getConfidence());
    SmartDashboard.putNumber("Red", colorSense.detectedColor.red);
    SmartDashboard.putNumber("green", colorSense.detectedColor.green);
    SmartDashboard.putNumber("blue", colorSense.detectedColor.blue);
    SmartDashboard.putNumber("Rotatinos",m_spinner.getColorSwitches());
    SmartDashboard.putString("colorkey", m_spinner.getCountColor().toString());
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

    // Updat color related value

    SmartDashboard.putString("colorFRC", colorSense.getColorString());
    SmartDashboard.putNumber("confidence", colorSense.getConfidence());
    SmartDashboard.putNumber("Red", colorSense.detectedColor.red);
    SmartDashboard.putNumber("green", colorSense.detectedColor.green);
    SmartDashboard.putNumber("blue", colorSense.detectedColor.blue);
    SmartDashboard.putNumber("Rotatinos",m_spinner.getColorSwitches());
    SmartDashboard.putString("colorkey", m_spinner.getCountColor().toString());
    
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

    
  }
}
