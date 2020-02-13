/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.wheel;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.*;

import edu.wpi.first.wpilibj.I2C;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;

public class SenseColor extends SubsystemBase {
  /**
   * Creates a new SenseColor.
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);
  private int proximity = m_colorSensor.getProximity();
  private double IR = m_colorSensor.getIR();
  //private Color detectedColor = m_colorSensor.getColor();
  private final ColorMatch m_colorMatcher = new ColorMatch();

   

  //public ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

  private boolean isBlue = getRawColor() >= blueLowerBound && getRawColor() <= blueUpperBound;
  private boolean isRed = getRawColor() >= redLowerBound && getRawColor() <= redUpperBound;
  private boolean isYellow = getRawColor() >= yellowLowerBound && getRawColor() <= yellowUpperBound;
  private boolean isGreen = getRawColor() >= greenLowerBound && getRawColor() <= greenUpperBound;

  private Colour prevColor = Colour.YELLOW;

  public enum Colour {

    RED(redLowerBound, redUpperBound ,1,'R') {
      public Colour next() {return YELLOW;}},

    YELLOW(yellowLowerBound, yellowUpperBound,2,'Y') {
      public Colour next() {return BLUE;}},

    BLUE(blueLowerBound, blueUpperBound,3,'B') {
      public Colour next() {return GREEN;}},

    GREEN(greenLowerBound, greenUpperBound,4,'G') {
      public Colour next() {return YELLOW;}};

    private final double upper;
    private final double lower;
    private final int position;
    private final char capital;

    public abstract Colour next();

    public char getCapital(){
      return capital;
    }

    public double getLower() {
      return lower;
    }

    public double getUpper() {
      return upper;
    }

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
    
	  

	public static Colour fromChar (final char Ch){
      switch(Ch){
        case('B'):
          return Colour.BLUE;
        case('Y'):
          return Colour.YELLOW;
        case('G'):
          return Colour.GREEN;
        case('R'):
          return Colour.RED;
        default:
          return Colour.YELLOW;
      }

    }
  
    private Colour(final double lowerBound, final double upperBound, final int position, final char capital) {
      this.upper = upperBound;
      this.lower = lowerBound;
      this.position = position;
      this.capital = capital;

    } 
  }   


  public boolean getIsBlue(){
    isBlue = getRawColor() >= Colour.BLUE.lower && getRawColor() <= Colour.BLUE.upper;

    return isBlue; 
  }

 public boolean getIsRed(){

  isRed = getRawColor() >= Colour.RED.lower && getRawColor() <= Colour.RED.upper; 

  return isRed; 

 }

 public boolean getIsYellow(){

  isYellow = getRawColor() >= Colour.YELLOW.lower && getRawColor() <= Colour.YELLOW.upper; 

  return isYellow; 

 }

 public boolean getIsGreen(){

  isGreen = getRawColor() >= Colour.GREEN.lower && getRawColor() <= Colour.GREEN.upper; 

  return isGreen; 

 }

/*
  public Color getColor() {
    return detectedColor;
  }
*/
  public double getRawColor() {
    return IR;
  }

  public int getProximity() {
    return proximity;
  }

 
  public Colour getColour(){
    if (getIsBlue()) {
      prevColor = Colour.BLUE;
      return Colour.BLUE;

    } else if (getIsRed()) {
      prevColor = Colour.RED;
      return Colour.RED;

    } else if (getIsGreen()) {
      prevColor = Colour.GREEN;
      return Colour.GREEN;

    } else if (getIsYellow()) {
      prevColor = Colour.YELLOW;
      return Colour.YELLOW;

    } else {
      return prevColor;

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

  public char getColorChar() {

    switch(getColour()){
      case YELLOW:
        return 'Y';
      case RED:
        return 'R';
      case BLUE:
        return 'B';
      case GREEN:
        return 'G';
      default:
        return 'Y';
    }
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
   // detectedColor = m_colorSensor.getColor();
    IR = m_colorSensor.getIR();
    proximity = m_colorSensor.getProximity();
    //match = m_colorMatcher.matchClosestColor(detectedColor);
  }

}
