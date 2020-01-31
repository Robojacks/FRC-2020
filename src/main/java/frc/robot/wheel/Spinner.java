/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.wheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wheel.SenseColor.*;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.platform.DeviceType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.*;

public class Spinner extends SubsystemBase {

  private final SenseColor colorSense = new SenseColor();
  private final Colour colorSensed = colorSense.getColour();
  private final CANSparkMax SpinnerMotor = new CANSparkMax(kSpinnerPort, MotorType.kBrushless);
  private final CANPIDController spinController = SpinnerMotor.getPIDController();
  /*DriverStation.getInstance().getGameSpecificMessage();*/
 

  public Spinner() {
    spinController.setP(spinnerWheel.Kp);
    spinController.setI(spinnerWheel.Ki);
    spinController.setD(spinnerWheel.Kd);
  }

  public void toSelectedColor(String message) {
    Colour objective = Colour.fromChar(message.charAt(0)).nextIn(2);
    SpinnerMotor.set(0.1);
    if ( colorSense.getColorChar() == objective.getCapital() ){
      SpinnerMotor.set(0);
    }
    
  }


  public void measuredSpin(final double rotations) {
    spinController.setReference(rotations, ControlType.kPosition);  

  }

  public void toSelectedRotation_Color(int nSpin){
    
    

  
  }
 

  public void toSelectedRotation_Ratio(int nSpin){

  }

  @Override
  public void periodic() {
  }

}

