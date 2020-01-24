// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.auton.*;
import frc.subsystem.*;
//import frc.robot.subsystem.Drive;
import frc.utility.math.*;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import java.util.concurrent.*;


import frc.utility.ThreadScheduler;
import frc.utility.Controller;
import frc.utility.VisionTarget;

public class Robot extends TimedRobot {
  //Controllers
  public Controller xbox = new Controller(0);
  public Controller leftStick = new Controller(1);
  public Controller rightStick = new Controller(2);

  //Subsystems 
  RobotTracker robotTracker = RobotTracker.getInstance();
  Drive drive = Drive.getInstance();

  //Multithreading stuff
  ExecutorService executor = Executors.newFixedThreadPool(2); //More than 2 threads is redundant as roborio only has two cores
  ThreadScheduler scheduler = new ThreadScheduler();
  Thread auto;

  

  private Integer m_autoSelected;
  private final SendableChooser<Integer> m_chooser = new SendableChooser<Integer>();

  @Override
  public void robotInit() {
    drive.calibrateGyro();
    //Schedule subsystems
    scheduler.schedule(drive, executor);
    scheduler.schedule(robotTracker, executor);
    m_chooser.addOption("idk", 0);
  }

  @Override
  public void robotPeriodic() {
  }

  boolean autoDone;
  @Override
  public void autonomousInit() {
    scheduler.resume();
    m_autoSelected = m_chooser.getSelected();
    AutoRoutine option = AutoRoutineGenerator.shit();
    auto = new Thread(option);
    auto.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
  }

  @Override 
  public void teleopInit() {
    if(auto != null)
      auto.interrupt();
    System.out.println("teleop init!");
    scheduler.resume();
  }

  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
  }
  @Override
  public void testPeriodic() {
  
  }

  @Override
  public void disabledInit() {
    if(auto != null){
      auto.interrupt();
    }
  }
  
  @Override
  public void disabledPeriodic() {
  }
}
