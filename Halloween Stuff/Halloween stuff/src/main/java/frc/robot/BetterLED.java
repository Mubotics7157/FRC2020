package frc.robot;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;


public class BetterLED{
    Spark LED;
    ArrayList <Double> colorCombos;
    double index;
    boolean ascending;
    double colorRamp;
    Timer timer;

    public BetterLED(){
        LED = new Spark(0);
        colorCombos = new ArrayList<>();
        timer = new Timer();
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
    
  /*  public void resetTimer(){
        timer.reset();
    }
*/
    public void generateRandLED(){
        if(timer.get()>= 5){
        index = Math.round(Math.random()*colorCombos.size()-1 +0);
        LED.set(colorCombos.get((int) index));
        timer.reset();
        }
    }

    public void setSpecColor(double pattern){
        LED.set(pattern);
    }

    public void rampColors() {
        ascending = colorRamp >= 1 ? false : true;
        colorRamp += ascending ? 0.0004 : -0.0004;
        LED.set(colorRamp);
      }
    
}