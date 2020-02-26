// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.auton.AutoRoutine;
import frc.auton.AutoRoutineGenerator;
import frc.subsystem.*;
//import frc.robot.subsystem.Drive;
import edu.wpi.first.wpilibj.TimedRobot;

import java.time.Duration;
import java.util.concurrent.*;


import frc.utility.ThreadScheduler;
import frc.utility.Controller;

public class Robot extends TimedRobot {
  //Controllers
  public Controller xbox = new Controller(0);
  public Controller leftStick = new Controller(1);
  public Controller rightStick = new Controller(2);

  //Subsystems 
  RobotTracker robotTracker = RobotTracker.getInstance();
  Drive drive = Drive.getInstance();
  //VisionManager vision = VisionManager.getInstance();
  Turret turret = Turret.getInstance();

  //Multithreading stuff
  ExecutorService executor = Executors.newFixedThreadPool(2); //More than 2 threads is redundant as roborio only has two cores
  ThreadScheduler scheduler = new ThreadScheduler();
  Thread auto;

  

  private Integer m_autoSelected;
  private final SendableChooser<Integer> m_chooser = new SendableChooser<Integer>();
  Trajectory traj2;

  @Override
  public void robotInit() {
    drive.calibrateGyro();
    drive.setPeriod(Duration.ofMillis(5));
    robotTracker.setPeriod(Duration.ofMillis(5));
    turret.setPeriod(Duration.ofMillis(20));

    //Schedule subsystems
    scheduler.schedule(drive, executor);
    scheduler.schedule(robotTracker, executor);
    scheduler.schedule(turret, executor);
    //scheduler.schedule(vision, executor);
    m_chooser.addOption("idk", 0);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoderL", Drive.getInstance().getLeftEncoderDistance());
    SmartDashboard.putNumber("encoderR", Drive.getInstance().getRightEncoderDistance());
  }

  boolean autoDone;
  @Override
  public void autonomousInit() {
    scheduler.resume();
    // m_autoSelected = m_chooser.getSelected();
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
    if(auto != null)
      auto.interrupt();
    System.out.println("teleop init!");
    scheduler.resume();
    //Drive.getInstance().setTeleOp();
  }

  @Override
  public void teleopPeriodic() {
    //drive.driveTeleOp(leftStick.getY(), rightStick.getY());
    //drive.tankDriveVelocity(0.2, 0.2);
    //if(xbox.getRawButtonPressed(1)){
      //System.out.println("Setting TurretState to FieldLock");
      //turret.setFieldLock();
    //}
  }

  @Override
  public void testInit() {
  }
  @Override
  public void testPeriodic() {
    drive.shift(1);
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
