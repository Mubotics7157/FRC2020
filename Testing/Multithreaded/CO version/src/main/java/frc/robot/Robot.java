// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.util.Units;
import frc.auton.*;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.subsystem.*;
//import frc.robot.subsystem.Drive;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

import java.time.Duration;
import java.util.ArrayList;
import java.util.List;
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
  //Turret turret = Turret.getInstance();

  //Multithreading stuff
  ExecutorService executor = Executors.newFixedThreadPool(2); //More than 2 threads is redundant as roborio only has two cores
  ThreadScheduler scheduler = new ThreadScheduler();
  Thread auto;

  

  private Integer m_autoSelected;
  private final SendableChooser<Integer> m_chooser = new SendableChooser<Integer>();

  @Override
  public void robotInit() {
    drive.calibrateGyro();
    drive.setPeriod(Duration.ofMillis(20));
    robotTracker.setPeriod(Duration.ofMillis(20));

    //Schedule subsystems
    scheduler.schedule(drive, executor);
    scheduler.schedule(robotTracker, executor);
    //scheduler.schedule(turret, executor);
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
    // AutoRoutine option = AutoRoutineGenerator.generate3();
    // auto = new Thread(option);
    // auto.start();
  TrajectoryConfig config = new TrajectoryConfig(4, 1);
  config.addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT);
  config.setKinematics(DriveTrainConstants.DRIVE_KINEMATICS);
  DifferentialDriveKinematicsConstraint kkk = new DifferentialDriveKinematicsConstraint(DriveTrainConstants.DRIVE_KINEMATICS, 3);
  config.addConstraint(kkk);
  config.setReversed(false);
  robotTracker.setOdometry(new Pose2d(0, 0, new Rotation2d(0)));
  Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
    // Start at the origin facing the +X direction
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass through these two interior waypoints, making an 's' curve path
    List.of(
        new Translation2d(2, 0),
        new Translation2d(3, 2),
        new Translation2d(4, -2),
        new Translation2d(5, 0)
    ),
    // End 3 meters straight ahead of where we started, facing forward
    new Pose2d(0, 0, new Rotation2d(0)),
    // Pass config
    config);
    drive.setAutoPath(exampleTrajectory);
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
    Drive.getInstance().setTeleOp();
  }

  @Override
  public void teleopPeriodic() {
    //drive.driveTeleOp(leftStick.getY(), rightStick.getY());
    drive.tankDriveVelocity(1, 1);
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
