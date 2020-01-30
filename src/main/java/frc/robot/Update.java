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
import static frc.robot.Constants.*;
/**
 * Mainly for logging values that need adjustment
 */
public class Update {

    // PID Controller Gains
    private double aP = angleCorrection.Kp;
    private double aI = angleCorrection.Ki;
    private double aD = angleCorrection.Kd;

    private double dP = distanceCorrection.Kp;
    private double dI = distanceCorrection.Ki;
    private double dD = distanceCorrection.Kd;
    // Starting positions
    private final Pose2d left = new Pose2d(-1, 0, Rotation2d.fromDegrees(0));
    private final Pose2d center = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
    private final Pose2d right = new Pose2d(1, 0, Rotation2d.fromDegrees(0));

    private static final SendableChooser choosePosition = new SendableChooser<>();

    public Update() {
        choosePosition.setDefaultOption("Center", center);
        choosePosition.addOption("Left", left);
        choosePosition.addOption("Right", right);
        SmartDashboard.putData("Starting Position", choosePosition);

        //display PID values (angle)
        SmartDashboard.putNumber("P value(angle)", angleCorrection.Kp);
        SmartDashboard.putNumber("I value(angle)", angleCorrection.Ki);
        SmartDashboard.putNumber("D value(angle)", angleCorrection.Kd);

        //display PID values (distance)
        SmartDashboard.putNumber("P value(distance)", distanceCorrection.Kp);
        SmartDashboard.putNumber("I value(distance)", distanceCorrection.Ki);
        SmartDashboard.putNumber("D value(distance)", distanceCorrection.Kd);
    }

    public static Pose2d getStartingPose() {
        final Pose2d position = (Pose2d) choosePosition.getSelected();
        return position;
    }

  public void logContinuous() {
    //change PID values for angle
    if ( aP != SmartDashboard.getNumber("P value(angle)", angleCorrection.Kp))  {
      aP = SmartDashboard.getNumber("P value(angle)", angleCorrection.Kp);
     }
    if ( aI != SmartDashboard.getNumber("I value(angle)", angleCorrection.Ki))  {
      aI = SmartDashboard.getNumber("I value(angle)", angleCorrection.Ki);
     }
    if ( aD != SmartDashboard.getNumber("D value(angle)", angleCorrection.Kd))  {
      aD = SmartDashboard.getNumber("D value(angle)", angleCorrection.Kd);
     } 

    //change PID values for distance
    if ( dP != SmartDashboard.getNumber("P value(distance)", distanceCorrection.Kp))  {
      dP = SmartDashboard.getNumber("P value(distance)", distanceCorrection.Kp);
     }
    if ( dI != SmartDashboard.getNumber("I value(distance)", distanceCorrection.Ki))  {
      dI = SmartDashboard.getNumber("I value(distance)", distanceCorrection.Ki);
     }
    if ( dD != SmartDashboard.getNumber("D value(distance)", distanceCorrection.Kd))  {
      dD = SmartDashboard.getNumber("D value(distance)", distanceCorrection.Kd);
     } 
  }

}
