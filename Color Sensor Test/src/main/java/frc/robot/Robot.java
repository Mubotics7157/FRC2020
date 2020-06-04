  /*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Spark;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.I2C; 
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.command.WaitCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  public TalonFX talon = new TalonFX(0); // change later
  boolean start = true;
  private final portMap map = new portMap();
  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  public final ColorSensorV3 m_ColorSensor = new ColorSensorV3(i2cPort);
  ColorMatch m_colorMatcher = new ColorMatch();
  private final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
  private final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
  private final Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);

  private final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
  final Timer timer = new Timer();
  public Joystick controller = new Joystick(map.controller);
  public ControlPanel control = new ControlPanel();
  boolean projectSensedColor = false;

  public LED lights=new LED();

  @Override
  public void robotInit() 
  {
    
    lights.init();
    control.init();
    //l.init();h
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
String goal;
  @Override
  public void teleopPeriodic() 
  {
 /*   if(controller.getRawButtonPressed(map.greenButton))
    {
      control.setGoalColor(kGreenTarget);
      goal = "green";
    }
   else if(controller.getRawButtonPressed(map.blueButton))
    {
      control.setGoalColor(kBlueTarget);
      goal = "blue";
    }
    else if(controller.getRawButtonPressed(map.yellowButton))
    {
      control.setGoalColor(kYellowTarget);
      goal ="yellow";  
    }
    else if(controller.getRawButtonPressed(map.redButton))
    {
      control.setGoalColor(kRedTarget);
      goal = "red";
    }*/

    control.receiveColor();
    
    control.getCurrentColor();
    control.rotationControl();
    control.printColor();
    control.colorControl();
    //control.printColor();    

    if(controller.getRawButtonPressed(5))
    projectSensedColor=true;
    else if (controller.getRawButtonPressed(6))
    projectSensedColor = false;
    if(projectSensedColor)
    {
    control.getCurrentColor();
    control.ledSense(lights);
    }
    if(controller.getRawButtonPressed(map.greenButton))
    {
      SmartDashboard.putNumber("Works", 1.0);
      lights.set(.77); //.77
      
    }
   
    if(controller.getRawButtonPressed(map.redButton))
    {
      lights.set(.61);
    }

    control.getCurrentColor();
       
    
    if(start)
    {
    lights.putTeamColors();
    timer.reset();
    timer.start();
    start = false;
    }
    if (timer.get()>=3.5)
    {
      lights.putTeamColors();
      timer.reset();
      timer.start();
    }

  }

  @Override
  public void testInit() {  
  }

  @Override
  public void testPeriodic() {
  }

}
