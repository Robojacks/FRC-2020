/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.wheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;
import frc.robot.Constants;

import java.lang.annotation.Target;

import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class SenseColor extends SubsystemBase {
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private int proximity = m_colorSensor.getProximity();
  private double IR = m_colorSensor.getIR();
  public Color detectedColor = m_colorSensor.getColor();
  private final ColorMatch m_colorMatcher = new ColorMatch();
  public ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);
  
  public SenseColor(){
    m_colorMatcher.addColorMatch(Colour.BLUE.target);
    m_colorMatcher.addColorMatch(Colour.YELLOW.target);
    m_colorMatcher.addColorMatch(Colour.RED.target);
    m_colorMatcher.addColorMatch(Colour.GREEN.target);
  }
  
  private Colour prevColor = Colour.YELLOW;

  public enum Colour {

    RED( Constants.kRedTarget ,0,"R"), 
      
    YELLOW(Constants.kYellowTarget,1,"Y"),

    BLUE(Constants.kBlueTarget,2,"B"),

    GREEN(Constants.kGreenTarget,3,"G");

    private final int position;
    private final String capital;
    private final Color target; 
   

    Colour (final Color Target, final int position, final String capital) {
      this.target= Target;
      this.position = position;
      this.capital = capital;
    }

    /**
     * 
     * @param n = number of ofset color counterclockwise
     * @return 
     */
    public Colour nextIn(int n) {
      n = this.position + n % 4;

      if (n == YELLOW.position) {
        return YELLOW;
      }

      else if (n == BLUE.position) {
        return BLUE;
      }

      else if (n == GREEN.position) {
        return GREEN;
      }

      else if (n == RED.position) {
        return RED;
      }

      else {
        return null;
      }
    
    }

  public String getCapital() {
    return capital;
  }

	public static Colour fromString (final String Ch){
    switch(Ch){
    case("B"):
      return Colour.BLUE;
    case("Y"):
      return Colour.YELLOW;
    case("G"):
      return Colour.GREEN;
    case("R"):
      return Colour.RED;
    default:
      return Colour.YELLOW;
    }

  }

}
  
  public double getRawColor() {
    IR = m_colorSensor.getIR();
    return IR;
  }

  public int getProximity() {
    return proximity;
  }

 
  public Colour getColour(){
    
    ColorMatchResult matching = m_colorMatcher.matchClosestColor(detectedColor);

    if (matching.color == Colour.BLUE.target) {
      prevColor = Colour.BLUE;
      return Colour.BLUE;

    } else if (matching.color ==  Colour.RED.target) {
      prevColor = Colour.RED;
      return Colour.RED;

    } else if (matching.color ==  Colour.GREEN.target) {
      prevColor = Colour.GREEN;
      return Colour.GREEN;

    } else if (matching.color ==  Colour.YELLOW.target) {
      prevColor = Colour.YELLOW;
      return Colour.YELLOW;

    } else {
      return Colour.BLUE;
    }

  }

  public Colour getPrevColour() {
    return prevColor;
  }


  public String getColorString() {
    
    switch(getColour()){
      case YELLOW:
        return "Yellow";
      case RED:
        return "Red";
      case BLUE:
        return "Blue";
      case GREEN:
        return "Green";
      default:
        return "Yellow";
    }

  }
   public double getConfidence(){
     return m_colorMatcher.matchClosestColor(detectedColor).confidence;
   }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    detectedColor = m_colorSensor.getColor();
    IR = m_colorSensor.getIR();
    proximity = m_colorSensor.getProximity();
  }

}
