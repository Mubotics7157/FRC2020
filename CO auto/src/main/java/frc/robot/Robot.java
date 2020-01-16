// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.tables.ITable;
import frc.auton.*;
import frc.subsystem.*;
//import frc.robot.subsystem.Drive;
import frc.utility.math.*;
import frc.utility.telemetry.TelemetryServer;
import frc.utility.control.motion.Path;
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
  TelemetryServer telemetryServer = TelemetryServer.getInstance();
  RobotTracker robotTracker = RobotTracker.getInstance();
  Drive drive = Drive.getInstance();

  //Multithreading stuff
  ExecutorService executor = Executors.newFixedThreadPool(2); //More than 2 threads is redundant as roborio only has two cores
  ThreadScheduler scheduler = new ThreadScheduler();
  Thread auto;

  

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<String>();

  @Override
  public void robotInit() {
    drive.calibrateGyro();
    //Schedule subsystems
    scheduler.schedule(drive, executor);
    scheduler.schedule(robotTracker, executor);
    
    drive.setSimpleDrive(true);
  }

  @Override
  public void robotPeriodic() {
  }

  boolean autoDone;
  @Override
  public void autonomousInit() {
    scheduler.resume();
    AutoRoutine option = AutoRoutineGenerator.shit();
    auto = new Thread(option);
    auto.start();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    robotTracker.debugOdometry();
    drive.debugSpeed();
  }

  @Override 
  public void teleopInit() {
    if(auto != null)
      auto.stop();
    System.out.println("teleop init!");
    scheduler.resume();
    drive.getFFGraph();
  }

  @Override
  public void teleopPeriodic() {
      xbox.update();
      leftStick.update();
      rightStick.update();

      if(xbox.getRawButton(2)){
        drive.debugDriveFF(0, 0);
        drive.debugSpeed();
      }else if(xbox.getRawButton(1)){
        drive.debugDriveFF(10, 10);
        drive.debugSpeed();
      }else if(xbox.getRawButton(3)){
        drive.debugDriveFF(-10, -10);
      }else {
        drive.setSimpleDrive(true);
        drive.jankDrive(xbox.getRawAxis(1), xbox.getRawAxis(5));
      }
      drive.debugDistance();
      drive.debugSpeed();
      robotTracker.debugOdometry();
      drive.debugVoltage();
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
      auto.stop();
    }
    drive.configMotors();
  }
  
  @Override
  public void disabledPeriodic() {
  }
}
