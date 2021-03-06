// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.auton.AutoRoutine;
import frc.auton.AutoRoutineGenerator;
import frc.robot.Constants.ShooterConstants;
import frc.subsystem.*;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
//import frc.robot.subsystem.Drive;
import edu.wpi.first.wpilibj.TimedRobot;

import java.time.Duration;
import java.util.concurrent.*;



import frc.utility.ThreadScheduler;

//import frc.utility.vanity.AddressableLEDs;
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
  //private AddressableLEDs harshalsWillie = new AddressableLEDs();

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
    c.start();
    scheduler.resume();
    // m_autoSelected = m_chooser.getSelected();
    turret.setLight(true);
    AutoRoutine option = AutoRoutineGenerator.redOne();//AutoRoutineGenerator.getRoutine(m_chooser.getSelected());
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
    //c.start();
    if(auto != null)
      auto.interrupt();
    System.out.println("teleop init!");
    scheduler.resume();
    turret.setLight(true);
    //turret.setTargetLock();
    //drive.setAutoPath(traj2);
    Drive.getInstance().setTeleOp();
    indexer.setLemons(1000);
    robotTracker.resetOdometry();
    turret.setDebug();
  }

  @Override
  public void teleopPeriodic() {
    drive.driveTeleOp(leftStick.getY(), rightStick.getY());
    if(Math.abs(xbox.getRawAxis(4)) > 0.3) {
      
        indexer.toggleHungry();
      //indexer.setHungry(true);
      //indexer.setIntakeSpeed(-xbox.getRawAxis(4));
      //indexer.setIntakeSpeed(1.0);
    }
    else if (xbox.getRawButton(1)) {
      indexer.setShooting();
    }
    else if (xbox.getRawButton(10)) {
      indexer.setPuke();
    }//else if(xbox.getRawButton(2)){
      else if(rightStick.getRawButton(1)){
        indexer.setShooting();
    }
    else{
      indexer.setHungry(false);
    }

    if (xbox.getRawButtonPressed(7)) {
      turret.setDebug();
    }
    /*else if (xbox.getRawButtonPressed(6)) {
      turret.setTargetLock();
    }*/
    if(leftStick.getRawButtonPressed(2))
      turret.setTargetLock();

    if(rightStick.getRawButtonPressed(2))
      turret.setFieldLock();

    indexer.setRPMAdjustment(leftStick.getRawAxis(3) * -200, leftStick.getRawAxis(3) * -200 / indexer.getRPMRatio());

    if(xbox.getRawButtonPressed(9)){
      indexer.setRPMRatio(ShooterConstants.RATIO_FLOATY);
    }else if(xbox.getRawButtonPressed(8)){
      //indexer.setRPMRatio(ShooterConstants.RATIO_NORMAL);
    }

    if(xbox.getRawButtonPressed(5)) {
      indexer.setSalivation(false);
    }

    if(xbox.getRawButtonPressed(3)) {
      indexer.toggleRPMTolerance();
      indexer.toggleShooterAngle();
    }

    if(Math.abs(xbox.getRawAxis(1)) > 0.05) {
      turret.adjustDebugHeading(xbox.getRawAxis(0) * -0.2);
    }
 
    //if(xbox.getRawButton(8)) {
      if(leftStick.getRawButton(1)){
        indexer.setRevving();

    }
    
      if(leftStick.getRawButtonPressed(3))
        turret.setOff();  //else{
      //indexer.setSwallowing(false);
    //}

    //drive.tankDriveVelocity(0.2, 0.2);
    //if(xbox.getRawButtonPressed(1)){
      //System.out.println("Setting TurretState to FieldLock");
      //turret.setFieldLock();
    //}
    //indexer.testShoot();
  
    
    //indexer.shootArbitrary(SmartDashboard.getNumber("bottom", 0), SmartDashboard.getNumber("top", 0));
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
