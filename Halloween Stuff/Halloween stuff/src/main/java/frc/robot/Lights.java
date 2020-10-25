package frc.robot;

import java.util.ArrayList;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;

public class Lights {

  static ArrayList<Double> lightPatterns = new ArrayList<Double>();
  static Spark light;

  public Lights() {

    light = new Spark(0);

    lightPatterns.add(-0.99); //rainbow, rainbow palette
    lightPatterns.add(-0.97); //rainbow, party palette
    lightPatterns.add(-0.87); //confetti
    lightPatterns.add(-0.73); //sinelon, lava palette
    lightPatterns.add(-0.77); //sinelon, party palette 
    lightPatterns.add(-0.55); //twinkles, rainbow palette
    lightPatterns.add(-0.39); //color waves, lava palette
    lightPatterns.add(-0.45); //color waves, rainbow palette
    lightPatterns.add(-0.35); //larson scanner, red
    lightPatterns.add(-0.25); //heartbeat, red
    lightPatterns.add(-0.23); //heartbeat, blue
    lightPatterns.add(-0.11); //strobe, red
    lightPatterns.add(-0.03); //end to end blend to black
    lightPatterns.add(0.11); //breath fast
    lightPatterns.add(0.37); //sparkle, color 1 and 2
    lightPatterns.add(0.51); //twinkles, color 1 and 2
    lightPatterns.add(0.55); //sinelon, color 1 and 2
    lightPatterns.add(0.59); //dark red
    lightPatterns.add(0.61); //red
    lightPatterns.add(0.65); //orange
    lightPatterns.add(0.67); //gold
    lightPatterns.add(0.75); //dark green
    lightPatterns.add(0.77); //green
    lightPatterns.add(0.81); //blue green
    lightPatterns.add(0.85); //dark blue
    lightPatterns.add(0.91); //violet
    lightPatterns.add(0.93); //white
    lightPatterns.add(0.99); //black

  }

  public static void mixItUp() {
    //After 1 minute pattern will change

    Timer timer = new Timer();
    timer.start();

    if (timer.get() >= 60){
      setLightPattern();
      timer.reset();
    }
  }

  public static void setLightPattern() {

    int pRange = lightPatterns.size();
    int randomP = (int)(Math.random() * pRange + 0);
    double pattern = lightPatterns.get(randomP);
    
    light.set(pattern);

  }
  
}