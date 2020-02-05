/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  private WPI_TalonSRX leftLauncher = new WPI_TalonSRX(kLeftShooterWheelPort);
  private WPI_TalonSRX rightLauncher = new WPI_TalonSRX(kRightShooterWheelPort);

  private WPI_TalonSRX conveyor = new WPI_TalonSRX(kConveyorBelt);

  //private CANPIDController leftController = new CANPIDController(leftLauncher);
  //private CANPIDController rightController = new CANPIDController(rightLauncher);

  //private CANEncoder leftEncoder = leftLauncher.getEncoder();
  //private CANEncoder rightEncoder = rightLauncher.getEncoder();

  private double setpoint = 0;
  private double error = 0;

  /**
   * Creates a new Shooter.
   */
  public Shooter() {
    //leftController.setP(shooterLeftPID.Kp);
    //leftController.setI(shooterLeftPID.Ki);
    //leftController.setD(shooterLeftPID.Kd);    

    //rightController.setP(shooterRightPID.Kp);
    //rightController.setI(shooterRightPID.Ki);
    //rightController.setD(shooterRightPID.Kd);
  }
  
  public void setVoltage(double volts){
    leftLauncher.setVoltage(volts);
    rightLauncher.setVoltage(volts);
    conveyor.setVoltage(volts);
  }
  
  public void setVelocity(double rpm, double allowedError) {
    setpoint = rpm;
    error = allowedError;

    //leftController.setReference(rpm, ControlType.kVelocity);
    //rightController.setReference(-rpm, ControlType.kVelocity);
  }

  /**
   * Using both encoder velocities, this method will determine if the shooter is within an rpm
   * range suitable to start firing balls
   * @return If the speed is at the desired rpm range, true; otherwise, false.
   */

  /*
  public boolean inRange() {
    boolean leftGreaterThanMin = leftEncoder.getVelocity() >= setpoint - error;
    boolean leftLesserThanMax = leftEncoder.getVelocity() <= setpoint + error;

    boolean rightGreaterThanMin = rightEncoder.getVelocity() >= -setpoint - error;
    boolean rightLesserThanMax = rightEncoder.getVelocity() <= -setpoint + error;

    if (leftGreaterThanMin && leftLesserThanMax && rightGreaterThanMin && rightLesserThanMax) {
      return true;

    } else {
      return false;

    }
  }
*/
  @Override
  public void periodic() {
    //SmartDashboard.putNumber("Left Shooter Velocity", leftLauncher.getEncoder().getVelocity());
    //SmartDashboard.putNumber("Right Shooter Velocity", rightLauncher.getEncoder().getVelocity());
  }
}
