package frc.robot;

import java.util.ArrayList;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;



class ControlPanel 
 {  
   /*Fields*/
private final portMap map = new portMap();  
Color currentColor;
Color colorSensed;
Color lastColor;
private final I2C.Port i2cPort = I2C.Port.kOnboard;
public final ColorSensorV3 m_ColorSensor = new ColorSensorV3(i2cPort);
ColorMatch m_colorMatcher = new ColorMatch();
private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
public ArrayList <Color> sequence = new ArrayList<>();
//public CANSparkMax mechanism = new CANSparkMax(map.wheelTurner, MotorType.kBrushless);
//Variables
double numRotations = 0;
int totalRotations=4;
public Color goalColor;
boolean start = true;
boolean initRun = true;
Color trackedColor;
boolean colorChanged = false;
/*Constructor*/
  public ControlPanel()
  {
      
  }
public void init()
{
  //colors
  m_colorMatcher.addColorMatch(kBlueTarget);
  m_colorMatcher.addColorMatch(kGreenTarget);
  m_colorMatcher.addColorMatch(kRedTarget);
  m_colorMatcher.addColorMatch(kYellowTarget);
  //sequence arraylist
  sequence.add(kGreenTarget);
  sequence.add(kBlueTarget);
  sequence.add(kYellowTarget);
  sequence.add(kRedTarget);
  sequence.add(kGreenTarget);
  sequence.add(kBlueTarget);  
  sequence.add(kYellowTarget);
  sequence.add(kRedTarget);
}
//senses color
   public void getCurrentColor()
   {
     Color detected = m_ColorSensor.getColor();
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detected);
    if(lastColor==null)
    {
      currentColor=match.color;
      lastColor=match.color;

    }
   else if(match.color==sequence.get(findCell(currentColor)+1))
   {
    lastColor=currentColor;
    currentColor = match.color;
    colorChanged = true;
   }
   else
   {
     lastColor=lastColor;
     currentColor=currentColor;
     colorChanged = false;

   }
   
   }
   //don't use
public void setGoalColor(String inColor)
{

    switch(inColor)
    {
      case "red":
        goalColor=kRedTarget;
        break;
      case "blue":
        goalColor =kBlueTarget;
        break;
      case "green":
        goalColor = kGreenTarget;
        break;
      case "yellow":
        goalColor = kYellowTarget;
        break;
    }
}

 

//sequence indexer
public int findCell(Color color)
{
  int index=6;
  for(int i = 0; i<sequence.size(); i++)
  {
    if(color==sequence.get(i))
    {
      index=i;
    }
    

  }
  if(index>3)
  {
    index%=4;
  }
  return index;

}
  //positional control
public void colorControl()
{
  //mechanism.set(.1);
  String finished="unknown";
  //printColor();
  if (currentColor==goalColor)
  {
    //mechanism.set(0)
    finished = "yes";
  }
  else
  {
    finished = "no";
  }
  SmartDashboard.putString("finished", finished);
}


public void rotationControl()
{
  if(initRun)
  {
    trackedColor=currentColor;
    if(lastColor==trackedColor)
    {
      numRotations=0;
    }
    initRun = false;
  }
  else if(numRotations<=totalRotations)
  { 
    //mechanism.set(.1);
    if(lastColor==trackedColor&&colorChanged)
    {
      numRotations+=.5;
      SmartDashboard.putNumber("total rotations", numRotations );
    }
  else if (numRotations>totalRotations)
  {
    //mechaism.set(0);  
    SmartDashboard.putString("done", "done");
  }
  }
  
  

}
//makes led to color sensed
public void ledSense(LED colorLight)
{
  
  if(currentColor == kRedTarget)
  {
    colorLight.set(.61);
  }
  if(currentColor == kBlueTarget)
  {
    colorLight.set(.87);}
  if(currentColor == kGreenTarget)
  {colorLight.set(.77);}
  if(currentColor == kYellowTarget)
  {colorLight.set(.69);}

}
//prints color to smart dashboard
public void printColor()
  { 
    String colorString;
     String colorStringtwo;
    if (currentColor == kBlueTarget) 
    {
      colorString = "Blue";
    }
     else if (currentColor == kRedTarget) 
     {
   
      colorString = "Red";
    }
     else if (currentColor == kGreenTarget) 
     {
      colorString = "Green";
     }
     else if (currentColor == kYellowTarget) 
     {
      colorString = "yellow";
    }
    else 
    {
      colorString = "unkown";
    }


    if (lastColor == kBlueTarget) 
    {
      colorStringtwo = "Blue";
    }
     else if (lastColor == kRedTarget) 
     {
   
      colorStringtwo = "Red";
    }
     else if (lastColor == kGreenTarget) 
     {
      colorStringtwo = "Green";
     }
     else if (lastColor == kYellowTarget) 
     {
      colorStringtwo = "yellow";
    }
    else 
    {
      colorStringtwo = "unkown";
    }
      if(lastColor==currentColor|| lastColor == null)
      {
        SmartDashboard.putString("discard?", "no");
        SmartDashboard.putString("Color sensed", colorString);
        SmartDashboard.putString("Last xColor sensed", colorStringtwo);
      }

      else if(lastColor!=currentColor)
      {
        int lastC = findCell(lastColor);

          SmartDashboard.putString("discard?", "no");
          SmartDashboard.putString("Color Sensed", colorString);
          SmartDashboard.putString("Last Color sensed", colorStringtwo);
        
 
      }
     
    }

    //gets color from driver station
    public void receiveColor(){
      String gameData;
      gameData = DriverStation.getInstance().getGameSpecificMessage();
      if(gameData.length() > 0)
      {
          switch (gameData.charAt(0)){
              case 'B' :
                  setGoalColor("blue");
                  break;
              case 'G' :
                  setGoalColor("green");
                  break;
              case 'R' :
                  setGoalColor("red");
                  break;
              case 'Y' :
                  setGoalColor("yellow");
                  break;
              default :
                  //This is corrupt data
                  System.out.println("Corrupt data");
                  break;
          }
      }
      else
      {
          //Code for no data received yet
      }
  }
}   