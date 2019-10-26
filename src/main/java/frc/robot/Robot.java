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



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  Drive drive = Drive.getInstance();
  public Controller xbox = new Controller(0);
  public Controller leftStick = new Controller(1);
  public Controller rightStick = new Controller(2);

  TelemetryServer telemetryServer = TelemetryServer.getInstance();
  RobotTracker robotTracker = RobotTracker.getInstance();

  ExecutorService executor = Executors.newFixedThreadPool(2); //More than 2 threads is redundant as roborio only has two cores
  ThreadScheduler scheduler = new ThreadScheduler();
  Thread auto;

  

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<String>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    drive.calibrateGyro();
    scheduler.schedule(drive, executor);
    scheduler.schedule(robotTracker, executor);
    
    drive.setSimpleDrive(false);
  }

  @Override
  public void robotPeriodic() {
  }

  boolean autoDone;
  @Override
  public void autonomousInit() {
    autoDone = false;
    scheduler.resume();

    int autoDir = 1;
    double startPos = 48+18;
    
    AutoRoutine option = AutoRoutineGenerator.generate();
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
    System.out.println("teleop init!");
    scheduler.resume();
    drive.getFFGraph();
    drive.setRandomShit(0);
  }

  @Override
  public void teleopPeriodic() {
      xbox.update();
      leftStick.update();
      rightStick.update();
      
      if (xbox.getRawButton(1)) {
        drive.writeFFPeriodic();
      }

      if (xbox.getRawButtonReleased(1)) {
        drive.FFClose();
      }

      if(xbox.getRawButton(2)){
        drive.setRandomShit(6);
      }else if(xbox.getRawAxis(3)>0.5){
        drive.jankDrive(xbox.getRawAxis(1)/2, xbox.getRawAxis(5)/2);
      }
      else{
        drive.setRandomShit(0);
      }

      SmartDashboard.putNumber("Velocity Left", drive.getLeftSpeed());
      
      SmartDashboard.putNumber("Velocity Right", drive.getRightSpeed());
      
      //boolean quickTurn = xbox.getRawButton(1) || xbox.getRawButton(2);

      // ORANGE DRIVE
      // drive.orangeDrive(-xbox.getRawAxis(1), -xbox.getRawAxis(4),
      // xbox.getRawAxis(2) > .3);
      // drive.setWheelVelocity(new DriveVelocity(20, 20));

      // CHEESY DRIVE
      //drive.cheesyDrive(-xbox.getRawAxis(1), -xbox.getRawAxis(4), quickTurn);
      //if(quickTurn) {
      //  drive.arcadeDrive(-xbox.getRawAxis(1), -xbox.getRawAxis(4));			
      //} else {
      //  drive.arcadeDrive(-xbox.getRawAxis(1), 0.75 * -xbox.getRawAxis(4));				
      //}

      //TANK DRIVE
      //drive.jankDrive(xbox.getRawAxis(1)/2, xbox.getRawAxis(5)/2);

      SmartDashboard.putNumber("left encoder", drive.getLeftDistance());
      SmartDashboard.putNumber("right encoder", drive.getRightDistance());
      SmartDashboard.putNumber("right encoder v", drive.getRightSpeed());
  }

  @Override
  public void testInit() {
  }
  @Override
  public void testPeriodic() {
  
  }

  @Override
  public void disabledInit() {
  }
  
  @Override
  public void disabledPeriodic() {
  }
}
