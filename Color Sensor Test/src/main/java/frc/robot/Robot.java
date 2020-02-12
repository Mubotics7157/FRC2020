/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;
import edu.wpi.first.wpilibj.Spark;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import java.util.ArrayList;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
//import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  public final ColorSensorV3 m_ColorSensor = new ColorSensorV3(i2cPort);
  ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  public ArrayList <Color> sequence = new ArrayList<>();
  Spark ledLight = new Spark(0);
    
  
  public int findCell(Color color)
  {
    int i = 0;
    while(i<5)
    {
      if(sequence.get(i)==color)
      {
        break; 
      }
      i++;
    }
    return i;
  }
    
  public void putColorLight()
  {
    ledLight.set(.53);
    SmartDashboard.putString("lights", "successful");
    //WaitCommand timedLights = new WaitCommand(2);
    //ledLight.set(whatever color you want to put here (most likely white or orange depending on first color))
  }


  public void printColor()
  { Color detectedColor = m_ColorSensor.getColor();
     ColorMatchResult match = m_colorMatcher.matchClosestColor(m_ColorSensor.getColor());
     String colorString = " ";

     
    if (match.color == kBlueTarget) 
    {
      colorString = "Blue";
    }
     else if (match.color == kRedTarget) 
     {
      colorString = "Red";
    }
     else if (match.color == kGreenTarget) 
     {
      colorString = "Green";
    }
     else if (match.color == kYellowTarget) 
     {
      colorString = "Yellow";
    }
    else 
    {
      colorString = "unknown";
    }


      Color lastColor = match.color;
      ColorMatchResult currentColor = m_colorMatcher.matchClosestColor(m_ColorSensor.getColor());
      if(lastColor!=currentColor.color)
      {
        if(currentColor.color!=sequence.get(findCell(lastColor)+1))
        {
          SmartDashboard.putString("discard?", "yes");
        }
        else
        {
          SmartDashboard.putString("discard?", "no");
          SmartDashboard.putString("Color Sensed", colorString);
          SmartDashboard.putNumber("Red ",detectedColor.red);
          SmartDashboard.putNumber("blue: ",detectedColor.blue);
          SmartDashboard.putNumber("green: ",detectedColor.green);
          SmartDashboard.putNumber("Confidence: ", match.confidence);
        }

      
      
      }
    }
  
  @Override
  public void robotInit() 
  {
  
  //__________________________________________
    m_colorMatcher.addColorMatch(kBlueTarget);
    m_colorMatcher.addColorMatch(kGreenTarget);
    m_colorMatcher.addColorMatch(kRedTarget);
    m_colorMatcher.addColorMatch(kYellowTarget);
    sequence.add(kGreenTarget);
    sequence.add(kBlueTarget);
    sequence.add(kYellowTarget);
    sequence.add(kRedTarget);
    sequence.add(kGreenTarget);
    //sequence.add(kBlueTarget);
    //sequence.add(kYellowTarget);
    //sequence.add(kRedTarget);
    
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() 
  {
  }

  @Override
  public void teleopPeriodic() 
  {
    printColor();
    putColorLight(); 
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
