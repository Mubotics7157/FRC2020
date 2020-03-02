// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.auton.AutoRoutine;
import frc.auton.AutoRoutineGenerator;
import frc.subsystem.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
//import frc.robot.subsystem.Drive;
import edu.wpi.first.wpilibj.TimedRobot;

import java.time.Duration;
import java.util.concurrent.*;


import frc.utility.ThreadScheduler;
import frc.utility.shooting.ShotGenerator.BACKSPINRATIOS;
import frc.utility.Controller;

public class Robot extends TimedRobot {
  //Controllers
  public Joystick xbox = new Joystick(0);
  public static Joystick leftStick = new Joystick(1);
  public static Joystick rightStick = new Joystick(2);
  private Compressor c = new Compressor();

  //Subsystems 
  RobotTracker robotTracker = RobotTracker.getInstance();
  Drive drive = Drive.getInstance();
  //VisionManager vision = VisionManager.getInstance();
  Turret turret = Turret.getInstance();
  Indexer indexer = Indexer.getInstance();

  //Multithreading stuff
  ExecutorService executor = Executors.newFixedThreadPool(2); //More than 2 threads is redundant as roborio only has two cores
  ThreadScheduler scheduler = new ThreadScheduler();
  Thread auto;

  

  private Integer m_autoSelected;
  private final SendableChooser<Integer> m_chooser = new SendableChooser<Integer>();
  Trajectory traj2;

  @Override
  public void robotInit() {
    c.stop();
    drive.calibrateGyro();
    drive.setPeriod(Duration.ofMillis(20));
    robotTracker.setPeriod(Duration.ofMillis(5));
    turret.setPeriod(Duration.ofMillis(20));
    SmartDashboard.putNumber("bottom", 0);
    SmartDashboard.putNumber("top", 0);

    //Schedule subsystems
    scheduler.schedule(drive, executor);
    scheduler.schedule(robotTracker, executor);
    scheduler.schedule(turret, executor);
    scheduler.schedule(indexer, executor);
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
    turret.setLight(true);
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
    c.stop();
    if(auto != null)
      auto.interrupt();
    System.out.println("teleop init!");
    scheduler.resume();
    turret.setLight(true);
    //turret.setTargetLock();
    //drive.setAutoPath(traj2);
    Drive.getInstance().setTeleOp();
    indexer.setLemons(1000);
  }

  @Override
  public void teleopPeriodic() {
    //drive.driveTeleOp(leftStick.getRawAx, rightStick.getY());
    if(xbox.getRawAxis(2) > 0.9) {
      indexer.setHungry(true);
    }
    else if (xbox.getRawAxis(3) > 0.9) {
      indexer.setShooting(BACKSPINRATIOS.FLOATY);
    }
    else {
      indexer.setHungry(false);
    }

    if (xbox.getRawButtonPressed(1)) {
      turret.setFieldLock();
    }
    else if (xbox.getRawButtonPressed(2)) {
      turret.setTargetLock();
    }
    else if (xbox.getRawButtonPressed(3)) {
      turret.setOff();
    }
    //drive.tankDriveVelocity(0.2, 0.2);
    //if(xbox.getRawButtonPressed(1)){
      //System.out.println("Setting TurretState to FieldLock");
      //turret.setFieldLock();
    //}
    //indexer.testShoot();
  
    
    //indexer.shootArbitrary(3000, 3000);
    //indexer.runAll();


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
    turret.setLight(false);
    turret.setOff();
    indexer.setHungry(false);
  }
  
  @Override
  public void disabledPeriodic() {
  }
}
