/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.shooter;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.*;

public class Shooter extends SubsystemBase {
  private WPI_TalonSRX leftLauncher = new WPI_TalonSRX(kLeftShooterWheelPort);
  private WPI_TalonSRX rightLauncher = new WPI_TalonSRX(kRightShooterWheelPort);

  private WPI_TalonSRX conveyor = new WPI_TalonSRX(kConveyorBelt);

  public void setVoltage(double volts){
    leftLauncher.setVoltage(volts);
    rightLauncher.setVoltage(volts);
    conveyor.setVoltage(volts);
  }

  @Override
  public void periodic() {
  }
}
