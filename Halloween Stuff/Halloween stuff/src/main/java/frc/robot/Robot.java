/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; 


public class Robot extends TimedRobot {


  
 Joystick controller = new Joystick(0);
 Music music = new Music();
 //BetterLED ledLight = new BetterLED();
 //AdressableLED led = new AdressableLED();
Lights light = new Lights();
  
  @Override
  public void robotInit() {
   
    music.init();
   
  }

  
  @Override
  public void robotPeriodic() {
  }


  @Override
  public void autonomousInit() {
    
   
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    //hehe,, ,me ge,,i,,,,,,,,,,,,,,,,,,,,,zzz1
   
  }
  

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    music.periodic();
    if(controller.getRawButtonPressed(1)){
      music.loadMusic(1);
    }
    if(controller.getRawButtonPressed(2)){
      music.loadMusic(-1);
    }
    if(controller.getRawButtonPressed(3)){
      music.togglePause();
    }
      //ledLight.generateRandLED();
      light.mixItUp();
  }

  @Override
  public void testPeriodic() {
  } 
}
