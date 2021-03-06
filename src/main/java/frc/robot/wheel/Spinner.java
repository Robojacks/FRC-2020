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
  
  private double colorSwitches = 9;

  private Colour pastColor = Colour.YELLOW;
  private Colour countCol= Colour.GREEN; 

  public Spinner(SenseColor colorSensor) {
    colorSense = colorSensor;
  }

  public void move(double speed) {
    SpinnerMotor.set(speed);
  }

  public void toSelectedColor(String c) {
    Colour objective = Colour.fromString(c).nextIn(2);
    move(colorSpeed);

    if (colorSense.getColour() == objective){
      move(0);
      System.out.println("Release!");
    }
  }

  public double getColorSwitches() {
    return colorSwitches;
  }

  public void changeMaxSwitches(double maxColorSwitches) {
    colorSwitches = maxColorSwitches * 2 ;
  };


  public void setCountColor() {
	  countCol = colorSense.getColour();
  }


public Colour getCountColor(){
  return countCol;
}

  public void toSelectedColorSwitches(){
    
    move(-colorSwitchSpeed);
    pastColor = colorSense.getColour();
    
    if(colorSense.getColour() == Colour.BLUE && pastColor != colorSense.getColour())  {
      colorSwitches--;
      System.out.println("color passed");
     
    }

    if (colorSwitches <= 0){
      toSelectedColor(countCol.getCapital());
    }

  }



  @Override
  public void periodic() {
  }


}
