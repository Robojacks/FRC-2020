/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.wheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wheel.SenseColor.*;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;



import static frc.robot.Constants.*;

public class Spinner extends SubsystemBase {

  private SenseColor colorSense;
  private final WPI_TalonSRX SpinnerMotor = new WPI_TalonSRX(kSpinnerPort);
  private int rotation = 10 * 8;

  public Spinner(SenseColor colorSensor) {
    colorSense = colorSensor;
  }

  public void move(double speed) {
    SpinnerMotor.set(speed);
    System.out.println("move works");
  }

  public void toSelectedColor(String c) {
    Colour objective = Colour.fromString(c).nextIn(2);
    SpinnerMotor.set(0.1);

    if (colorSense.getColour() == objective){
      SpinnerMotor.set(0);
      System.out.println("Release!");
    }
  }

  public int getCurrentRotations() {
    return rotation;
  }

  public void changeMaxRotations(int maxRotations) {
    rotation = maxRotations;
  };

  public void toSelectedRotation_Color(){
    SpinnerMotor.set(rotationSpeed);

    if(colorSense.getColour() == colorSense.getPrevColour().next()) {
      rotation--;
    }

    if (rotation >= 0){
      SpinnerMotor.set(0);
    }
  
  }
 
  @Override
  public void periodic() {
  }

}

