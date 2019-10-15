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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

import java.util.concurrent.*;


import frc.utility.ThreadScheduler;
import frc.utility.Controller;
import frc.utility.JetsonUDP;
import frc.utility.VisionTarget;



/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  Drive drive = Drive.getInstance();
  public Controller xbox = new Controller(0);
  public Controller wheel = new Controller(3);
  public Controller stick = new Controller(1);
  public Controller buttonPanel = new Controller(2);

  TelemetryServer telemetryServer = TelemetryServer.getInstance();
  JetsonUDP jetsonUDP = JetsonUDP.getInstance();
  RobotTracker robotTracker = RobotTracker.getInstance();

  ExecutorService executor = Executors.newFixedThreadPool(2);
  ThreadScheduler scheduler = new ThreadScheduler();
  Thread auto;
  TemplateAuto option;

  boolean firstTeleopRun = true;

  

  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<String>();
  private final SendableChooser<String> dir_chooser = new SendableChooser<String>();
  private final SendableChooser<String> goodbad = new SendableChooser<String>();
  private final SendableChooser<String> start_chooser = new SendableChooser<String>();
  private final SendableChooser<String> red_blue = new SendableChooser<String>();

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    drive.calibrateGyro();
    m_chooser.addOption("Cargo F_F", "Cargo F_F");
    m_chooser.addOption("Cargo F_1", "Cargo F_1");
    m_chooser.addOption("Cargo 1_2", "Cargo 1_2");
    m_chooser.addOption("Rocket Mid Adaptive", "Rocket Mid Adaptive");

    m_chooser.setDefaultOption("Rocket Mid", "Rocket Mid");
    
    SmartDashboard.putData("Autonomous Mode", m_chooser);

    dir_chooser.setDefaultOption("Left", "Left");
    dir_chooser.addOption("Right", "Right");
    SmartDashboard.putData("Starting Side", dir_chooser);

    start_chooser.setDefaultOption("Lvl1", "Lvl1");
    start_chooser.addOption("Lvl2", "Lvl2");
    SmartDashboard.putData("Starting Height", start_chooser);

    red_blue.setDefaultOption("Red", "Red");
    red_blue.addOption("Blue", "Blue");
    SmartDashboard.putData("Red and Blue", red_blue);

    goodbad.setDefaultOption("good", "good");
    goodbad.addOption("bad","bad");
    goodbad.addOption("mindBuisness", "mindBuisness");
    goodbad.addOption("mInDbUiSnEsS", "mInDbUiSnEsS");
    SmartDashboard.putData("Good or Bad? To be or Not to Be?", goodbad);


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

    if(dir_chooser.getSelected().equals("Right")) autoDir = -1;
    else autoDir = 1;

    if(start_chooser.getSelected().equals("Lvl2")) startPos = 18+19-3;//-8;
    else startPos = 48+18;
    
    option = new TemplateAuto(new Translation2D(0,1),autoDir);
    auto = new Thread(option);
    auto.start();
    
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    buttonPanel.update();
    if(!autoDone) {
      if(buttonPanel.getRawButton(2)) {
        autoDone = true;
        teleopInit();
      }
    }
    if(autoDone) {
      teleopPeriodic();
    }
  } 

  public synchronized void killAuto() {
    if(option != null) {
      option.killSwitch();
    }

    if(auto != null) {
      while(auto.getState() != Thread.State.TERMINATED);
      drive.stopMovement();
      drive.setTeleop();
    }
    
  }

  @Override 
  public void teleopInit() {
    killAuto();
    System.out.println("teleop init!");
    scheduler.resume();
    firstTeleopRun = true;
  }

  @Override
  public void teleopPeriodic() {
      xbox.update();
      stick.update();
      buttonPanel.update();
      wheel.update();
      firstTeleopRun = false;
  }

  @Override
  public void testInit() {
   // drive.stopMovement();
   // scheduler.resume();
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
