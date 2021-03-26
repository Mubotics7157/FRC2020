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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
//import frc.robot.subsystem.Drive;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;

import java.time.Duration;
import java.util.concurrent.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.utility.ThreadScheduler;

//import frc.utility.vanity.AddressableLEDs;
public class Robot extends TimedRobot {
  //Controllers
  public Joystick xbox = new Joystick(0);
  public static Joystick leftStick = new Joystick(1);
  public static Joystick rightStick = new Joystick(2);
  private Compressor c = new Compressor();
  DigitalInput breakBeam = new DigitalInput(1);

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
    //c.start();
    scheduler.resume();
    // m_autoSelected = m_chooser.getSelected();
    turret.setLight(true);
    //drive.setTuning();
    robotTracker.resetOdometry();
    AutoRoutine option = AutoRoutineGenerator.barrelRoutine();//AutoRoutineGenerator.getRoutine(m_chooser.getSelected());
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
    SmartDashboard.putBoolean("did it pass", breakBeam.get());
    /*try {
				writeToFile(differentialDriveOdometry.getPoseMeters().getTranslation().getX(),
						differentialDriveOdometry.getPoseMeters().getTranslation().getY(),
						differentialDriveOdometry.getPoseMeters().getRotation().getDegrees());
			} catch (FileNotFoundException e) {
				e.printStackTrace();
			}*/
    /*if(Math.abs(xbox.getRawAxis(4)) > 0.3) {
      
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
    }*/

//shooting
    if(xbox.getRawButtonPressed(1))
    indexer.setRevving();
    if(Math.abs(xbox.getRawAxis(1)) > 0.05) {
     turret.adjustDebugHeading(xbox.getRawAxis(0) * -0.2);
    }  

// -- - - - - - -  - - - - -  everything to do with running intake

// running the belt - -- - -  - - 
    if(xbox.getRawButtonPressed(3)){
      indexer.sideChew();  
    }
    if(xbox.getRawButtonReleased(3)){
    }
// - - - - - - retract the intake (should auto deploy when set hungry) - - - - - -

    if(xbox.getRawButtonPressed(4)) {
      indexer.setSalivation(false);
    }

//  - - - - - - - - - - - -  runs the chute - - - - - - - - -  - - - 
      if(xbox.getRawButtonPressed(2))
        indexer.setIndexing(true);

      if(xbox.getRawButtonReleased(2))
        indexer.setIndexing(false);
// - - - - - - - - - - - - 



// - - - - - - setting different modes for the turret - - - - - - -
    if(xbox.getRawButton(5))
      turret.setFieldLock();

    if(xbox.getRawButtonPressed(6))
      turret.setTargetLock();
      
     if (xbox.getRawButtonPressed(7)) 
      turret.setDebug();
     
    if(xbox.getRawButtonPressed(8))
      turret.setOff();  

// - - - - - - - 
    indexer.setRPMAdjustment(leftStick.getRawAxis(3) * -200, leftStick.getRawAxis(3) * -200 / indexer.getRPMRatio());
    
      if(leftStick.getRawButtonPressed(1)){
        indexer.setHungry(true);

    }

    if(leftStick.getRawButtonReleased(1)){
      indexer.setHungry(false);
    }

       if(xbox.getRawAxis(2)<.05 && xbox.getRawAxis(2)>-.05){
         indexer.setOff();
       }

       else if(xbox.getRawAxis(2)>.05){
         indexer.setHungry(true);
       }

       else if(xbox.getRawAxis(2) < -.05){
         indexer.setPuke();
       }
          
      
      
        //else{
      //indexer.setSwallowing(false);
    //}


    /*if(xbox.getRawButtonPressed(3)) {
      indexer.toggleRPMTolerance();
      indexer.toggleShooterAngle();
    }*/

    /*if(xbox.getRawButtonPressed(9)){
      indexer.setRPMRatio(ShooterConstants.RATIO_FLOATY);
    }else if(xbox.getRawButtonPressed(8)){
      //indexer.setRPMRatio(ShooterConstants.RATIO_NORMAL);
    }*/

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
    drive.setBreakMode();
  }
  
  @Override
  public void disabledPeriodic() {
  }
}
