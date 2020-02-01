/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.climber;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANDigitalInput.LimitSwitch;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;



public class Arm extends SubsystemBase {
  private WPI_TalonSRX arm = new WPI_TalonSRX(0);

  public enum armState {
    REACH, PULL
  }

  // Keeps track of how high the shooter is
  private armState state = armState.PULL;

  public void reach(){
    arm.set(ControlMode.PercentOutput,armReachSpeed);
    state = armState.REACH;
  }

  public void pull(){
    arm.set(ControlMode.PercentOutput, armPullSpeed);
    state = armState.PULL;
  }

  public void switchArm() {
    switch(state) {
      case PULL:
        reach();
      case REACH:
        pull();
      default:
        pull();
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


}


