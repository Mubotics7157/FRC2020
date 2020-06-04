package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
class LED
{
   
    public ArrayList <Double> colorCombos = new ArrayList<>();
    Spark ledLight= new Spark(45); //placeholder
    private double colorRamp = 0;
    boolean ascending = false;
    

    public LED()
    {
        
    }


    
    public void putRandomLED()
    {
      double index = Math.random()*.9+-1.0;
      ledLight.set(index);
      SmartDashboard.putString("lights", "successful");
    }
  
    public void rampColors() {
      ascending = colorRamp >= 1 ? false : true;
      colorRamp += ascending ? 0.0004 : -0.0004;
      ledLight.set(colorRamp);
    }

    public void putTeamColors()
    {
        int index=colorCombos.size()-1;
        double value = Math.floor(Math.random()*index+0);
        ledLight.set(colorCombos.get((int)value));
        SmartDashboard.putNumber("index", colorCombos.get((int)value));
    }
    public void set(double num)
    {
      ledLight.set(num);
    }

    public void blowTheLoad(){ //( ͡° ͜ʖ ͡°)
      ledLight.set(.77);
    } 


  public void init()
  {
      colorRamp = 0;
      colorCombos.add(-.21);
      colorCombos.add(-.81);
      colorCombos.add(-.03);
      colorCombos.add(.05);
      colorCombos.add(.11);
      colorCombos.add(.17);
      colorCombos.add(.25);
      colorCombos.add(.31);
      colorCombos.add(.39);
      colorCombos.add(.41);
      colorCombos.add(.53);
      colorCombos.add(.55);
      colorCombos.add(.93);
      colorCombos.add(.63);
  }

}