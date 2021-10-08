// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.auton.AutoRoutine;
import frc.auton.AutoRoutineGenerator;
import frc.robot.Constants.ShooterConstants;
import frc.subsystem.*;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
//import frc.robot.subsystem.Drive;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import java.time.Duration;
import java.util.concurrent.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.unmanaged.Unmanaged;

import frc.utility.ThreadScheduler;

//import frc.utility.vanity.AddressableLEDs;
public class Robot extends TimedRobot {
  //Controllers
  public static Joystick xbox = new Joystick(0);
  public static Joystick leftStick = new Joystick(1);
  public static Joystick rightStick = new Joystick(2);
  private Compressor c = new Compressor();

  //Subsystems 
  RobotTracker robotTracker = RobotTracker.getInstance();
  Drive drive = Drive.getInstance();
  //VisionManager vision = VisionManager.getInstance();
  Turret turret = Turret.getInstance();
  Indexer indexer = Indexer.getInstance();
  //private AddressableLEDs harshalsWillie = new AddressableLEDs();

  //Multithreading stuff
  ExecutorService executor = Executors.newFixedThreadPool(2); //More than 2 threads is redundant as roborio only has two cores
  ThreadScheduler scheduler = new ThreadScheduler();
  climb climber = climb.getInstance();
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
    m_chooser.addOption("arbitrary", 0);
    //harshalsWillie.setLED();
    m_chooser.addOption("human player station", 2);
    m_chooser.setDefaultOption("target", 1);
    SmartDashboard.putData(m_chooser);
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("encoderL", Drive.getInstance().getLeftEncoderDistance());
    SmartDashboard.putNumber("encoderR", Drive.getInstance().getRightEncoderDistance());
  }

  boolean autoDone;
  @Override
  public void autonomousInit() {
    //c.start();
    scheduler.resume();
    // m_autoSelected = m_chooser.getSelected();
    turret.setLight(true);
    //drive.setTuning();
    robotTracker.resetOdometry();
    AutoRoutine option = AutoRoutineGenerator.simpleLine();
    //AutoRoutineGenerator.getRoutine(m_chooser.getSelected());
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
    c.start();
    if(auto != null)
      auto.interrupt();
    System.out.println("teleop init!");
    scheduler.resume();
    Drive.getInstance().setTeleOp();
    turret.setLight(true);
    indexer.setLemons(1000);
    robotTracker.resetOdometry();
    turret.setOff();
    turret.resetTurretPosition();
    climber.setManual();

  }
  /*
@Override
  public void teleopPeriodic() {


    if(leftStick.getRawButton(1)){
      indexer.setHungry(true);
    }

    else if(xbox.getRawButton(1)){
      indexer.setShooting();
    }

    else if(leftStick.getRawButton(3)){
      indexer.setPuke();
    }

    else if(rightStick.getRawButton(1)){
      indexer.setIndexing();
    }

    else{
      indexer.setHungry(false);
    }

    if(xbox.getRawButton(4)){
      indexer.setSalivation(true);
    }

    else if(rightStick.getRawButtonPressed(4)){
      //indexer.setSalivation(false);
      turret.setInnerPort();
    }

    else if(xbox.getRawButton(8)){
      indexer.setSwallowing(true);
    }

    else if(xbox.getRawAxis(4)> 0){
      indexer.toggleShooterAngle();
    }

    else if(xbox.getRawButton(9)){
      //indexer.toggleManualBeamBreak();
      indexer.setInnerPortMode();
    }


     //indexer.setRPMAdjustment(leftStick.getRawAxis(3)*-200, leftStick.getRawAxis(3)*-200/indexer.getRPMRatio()); 

// - - - - - - setting different modes for the turret - - - - - - -
    if(xbox.getRawButton(5))
      turret.setFieldLock();

    else if(xbox.getRawButtonPressed(6))
      turret.setTargetLock();
      
     else if(xbox.getRawButtonPressed(7)) 
      turret.setDebug();
     
    else if(xbox.getRawButtonPressed(2))
      turret.setOff();  


// - - - - - - - 
 if(Math.abs(xbox.getRawAxis(3)) > 0.25) {
      turret.adjustDebugHeading(xbox.getRawAxis(3) * -0.2);
    }

    indexer.setRPMAdjustment(leftStick.getRawAxis(3) * -200, leftStick.getRawAxis(3) * -200 / indexer.getRPMRatio());
  }
  */
 @Override
  public void teleopPeriodic() {

    
    if(xbox.getRawButton(8)){
      indexer.setHungry(true);
    }

    else if(xbox.getRawButton(9)){
      indexer.setShooting();
    }

    else if(leftStick.getRawButton(3)){
      indexer.setPuke();
    }

    else if(xbox.getRawButton(7)){
      indexer.setIndexing();
    }

    else{
      indexer.setHungry(false);
    }

    if(xbox.getRawButton(6)){
      indexer.setSalivation(true);
    }

    else if(xbox.getRawButton(5)){
      indexer.setSalivation(false);
    }

    else if(xbox.getRawButton(10)){
      indexer.setSwallowing(true);
    }
    

    if(xbox.getRawButtonPressed(2))
      turret.setTargetLock();
      
     else if(xbox.getRawButtonPressed(1)) 
      turret.setDebug();
     
    else if(xbox.getRawButtonPressed(3))
      turret.setOff();  

     if(Math.abs(xbox.getRawAxis(3)) > 0.25) {
      turret.adjustDebugHeading(xbox.getRawAxis(3) * -0.2);
    }

    indexer.setRPMAdjustment(leftStick.getRawAxis(3) * -200, leftStick.getRawAxis(3) * -200 / indexer.getRPMRatio());
    if(xbox.getRawButtonPressed(4)){
      indexer.setInnerPortMode();
    }
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
    drive.setNeutralMode(NeutralMode.Brake);
  }
  
  @Override
  public void disabledPeriodic() {
  }
}