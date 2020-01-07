/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drive extends SubsystemBase {
  private WPI_TalonFX leftFront = new WPI_TalonFX(Constants.kLeftFront);
  private WPI_TalonFX leftBack = new WPI_TalonFX(Constants.kLeftBack);
  private WPI_TalonFX rightFront = new WPI_TalonFX(Constants.kRightFront);
  private WPI_TalonFX rightBack = new WPI_TalonFX(Constants.kRightBack);

  private final SpeedControllerGroup leftMotors = new SpeedControllerGroup(leftFront, leftBack);
  private final SpeedControllerGroup rightMotors = new SpeedControllerGroup(rightFront, rightBack);
  
  private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);
  
  private final Encoder rightEncoder = new Encoder(Constants.kRightEncoder[0], Constants.kRightEncoder[1]);
  private final Encoder leftEncoder = new Encoder(Constants.kLeftEncoder[0], Constants.kLeftEncoder[1]);
  
  private final AHRS navx = new AHRS(Port.kMXP);
  private final DifferentialDriveOdometry odometry;

  private final boolean falcons = false;

  public Drive() {
    leftEncoder.setDistancePerPulse(Constants.kDriveEncoderPPR);
    rightEncoder.setDistancePerPulse(Constants.kDriveEncoderPPR);
    
    resetEncoders();
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
  }

  @Override
  public void periodic() {
    odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftDistance(), getRightDistance());
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocity(), getRightVelocity());
  }

  private double getRightDistance() {
    // Meters
    return rightEncoder.getDistance();
  }

  private double getLeftDistance() {
    // Meters
    return leftEncoder.getDistance();
  }

  private double getLeftVelocity() {
    // Meters / second
    return leftEncoder.getRate();
  }

  private double getRightVelocity() {
    // Meters / second
    return rightEncoder.getRate();
  }

  private double getHeading() {
    return navx.getYaw();
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  /**
   * Drives the robot using arcade controls.
   *
   * @param fwd the commanded forward movement
   * @param rot the commanded rotation
   */
  public void arcadeDrive(double fwd, double rot) {
    drive.arcadeDrive(fwd, rot);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(-rightVolts);
  }

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  public void resetEncoders() {
    leftEncoder.reset();
    rightEncoder.reset();
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistance() {
    return (getLeftDistance() + getRightDistance()) / 2.0;
  }

  /**
   * Sets the max output of the drive.  Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    drive.setMaxOutput(maxOutput);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return navx.getRate();
  }

  double Deadband(double value) {
		/* Upper deadband */
		if (value >= +0.05) 
			return value;
		
		/* Lower deadband */
		if (value <= -0.05)
			return value;
		
		/* Outside deadband */
		return 0;
	}
}
