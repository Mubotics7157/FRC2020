package frc.subsystem;

import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_LEFT_MASTER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_LEFT_SLAVE;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_RIGHT_MASTER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_RIGHT_SLAVE;
import static frc.robot.Constants.DriveTrainConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.DriveTrainConstants.FEED_FORWARD;
import static frc.robot.Constants.DriveTrainConstants.SENSOR_UNITS_PER_ROTATION;
import static frc.robot.Constants.DriveTrainConstants.WHEEL_CIRCUMFERENCE_METERS;
import static frc.robot.Constants.TeleConstants.MAX_ANGULAR_VEL;
import static frc.robot.Constants.TeleConstants.MAX_SPEED_TELE;
import static frc.robot.Constants.TrajectoryConstants;

import java.time.Duration;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;
import frc.utility.control.SynchronousPid;

/**
 * DriveTrainSubsystem
 */
public class Drive extends Threaded{

  private final TalonFX leftMaster = new TalonFX(DEVICE_ID_LEFT_MASTER);
  private final TalonFX leftSlave = new TalonFX(DEVICE_ID_LEFT_SLAVE);
  private final TalonFX rightMaster = new TalonFX(DEVICE_ID_RIGHT_MASTER);
  private final TalonFX rightSlave = new TalonFX(DEVICE_ID_RIGHT_SLAVE);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private RamseteController ramseteController = new RamseteController();
  private SynchronousPid turnPID = new SynchronousPid(0.01, 0, 0, 0);
  
	private static final Drive instance = new Drive();

	public static Drive getInstance() {
		return instance;
  }
  
	public enum DriveState {
		TELEOP, PUREPURSUIT, TURN, HOLD, DONE
  }
  
  DriveState driveState = DriveState.TELEOP;
  Trajectory currentTrajectory;
  Rotation2d wantedHeading;

  public Drive() {
    zeroDriveTrainEncoders();
    gyro.zeroYaw();

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative;
    talonConfig.neutralDeadband = DriveTrainConstants.DEADBAND;
    talonConfig.slot0.kP = DriveTrainConstants.kP;
    talonConfig.slot0.kI = 0.0;
    talonConfig.slot0.kD = DriveTrainConstants.kD;
    talonConfig.slot0.integralZone = 400;
    talonConfig.slot0.closedLoopPeakOutput = 1.0;
    talonConfig.closedloopRamp = DriveTrainConstants.CLOSED_LOOP_RAMP;
    talonConfig.openloopRamp = DriveTrainConstants.OPEN_LOOP_RAMP;

    rightMaster.configAllSettings(talonConfig);
    leftMaster.configAllSettings(talonConfig);
    leftMaster.setNeutralMode(NeutralMode.Coast);
    rightMaster.setNeutralMode(NeutralMode.Coast);
    rightSlave.setNeutralMode(NeutralMode.Coast);
    leftSlave.setNeutralMode(NeutralMode.Coast);

    leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
    
    rightMaster.setInverted(true);
    rightSlave.setInverted(true);
    rightMaster.setSensorPhase(true);
    leftMaster.setSensorPhase(true);
    rightMaster.overrideLimitSwitchesEnable(false);
    leftMaster.overrideLimitSwitchesEnable(false);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
  }

  @Override
  public void update() {
    DriveState snapDriveState;
      synchronized (this) {
        snapDriveState = driveState;
      }
      switch (snapDriveState) {
        case TELEOP:
          updateTeleOp();
          break;
        case PUREPURSUIT:
          updatePathController();
          break;
        case TURN:
          updateTurn();
          break;
        case HOLD:
          updateHold();
          break;
        case DONE:
          break;
      }
  }

  public synchronized void setAutoPath(Trajectory path) {
    currentTrajectory = path;
    driveState = DriveState.PUREPURSUIT;
    ramseteController.setTolerance(new Pose2d(new Translation2d(TrajectoryConstants.TOLERANCE_METERS, TrajectoryConstants.TOLERANCE_METERS)
      , Rotation2d.fromDegrees(TrajectoryConstants.TOLERANCE_DEGREES)));
    updatePathController();
  }
  
	public void setRotation(Rotation2d angle) {
		synchronized (this) {
			wantedHeading = angle;
			driveState = DriveState.TURN;
		}
  }

  private void updateHold() {
    leftMaster.set(ControlMode.Position, leftMaster.getSelectedSensorPosition());
    rightMaster.set(ControlMode.Position, rightMaster.getSelectedSensorPosition());
  }

  public void setHold() {
    synchronized (this) {
      driveState = DriveState.HOLD;
    }
  }

  private void updateTeleOp() {

  }

  public void driveTeleOp(double l, double r) {
    if (driveState == DriveState.TELEOP) {
      tankDrive(l, r);
    }
  }

  public void setTeleOp() {
    synchronized (this) {
      driveState = DriveState.TELEOP;
    }
  }
  
  private void updateTurn() {
		double error = wantedHeading.rotateBy(RobotTracker.getInstance().getOdometry().getRotation().unaryMinus()).getDegrees();
		double deltaSpeed;
		deltaSpeed = turnPID.update(error);
		deltaSpeed = Math.copySign(
		OrangeUtility.coercedNormalize(Math.abs(deltaSpeed), 0, 180, 0, TrajectoryConstants.MAX_SPEED_AUTO), deltaSpeed);
		if (Math.abs(error) < 10 && deltaSpeed < 0.2) {
			tankDriveVelocity(0, 0);
			synchronized (this) {
				driveState = DriveState.DONE;
			} 
		} else {
			tankDriveVelocity(-deltaSpeed, deltaSpeed);
		}
  }
  

  public void updatePathController() {
    Trajectory.State goal = currentTrajectory.sample(3.4);
    ChassisSpeeds adjustedSpeeds = ramseteController.calculate(RobotTracker.getInstance().getOdometry(), goal);
    DifferentialDriveWheelSpeeds wheelSpeeds = DRIVE_KINEMATICS.toWheelSpeeds(adjustedSpeeds);
    double left = wheelSpeeds.leftMetersPerSecond;
    double right = wheelSpeeds.rightMetersPerSecond;

    if (ramseteController.atReference()) {
      synchronized (this) {
        driveState = DriveState.DONE;
      }
    }
    tankDriveVelocity(left, right);
  }

  public boolean isFinished() {
    return driveState == DriveState.DONE;
  }

  /**
   * Drives the robot by adjusting x axis speed and z axis rotation
   * 
   * @param speed    speed along the x axis [-1.0..1.0]
   * @param rotation rotation rate along the z axis [-1.0..1.0]
   */
  public void arcadeDrive(double speed, double rotation) {
    arcadeDrive(speed, rotation, true);
  }

  /**
   * Drives the robot by adjusting x axis speed and z axis rotation
   * 
   * @param speed      speed along the x axis [-1.0..1.0]
   * @param rotation   rotation rate along the z axis [-1.0..1.0]
   * @param useSquares if set, decreases input sensitivity at low speeds
   */
  public void arcadeDrive(double speed, double rotation, boolean useSquares) {
    var xSpeed = speed * MAX_SPEED_TELE;
    var zRotation = Rotation2d.fromDegrees(-rotation * MAX_ANGULAR_VEL).getRadians();;
    if (useSquares) {
      xSpeed *= Math.abs(xSpeed);
      zRotation *= Math.abs(zRotation);
    }
    var wheelSpeeds = DRIVE_KINEMATICS.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, zRotation));
    tankDriveVelocity(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  /**
   * Drives the robot by individually addressing the left and right side of the
   * drive train
   * 
   * @param leftSpeed  speed of the left motors [-1.0..1.0]
   * @param rightSpeed speed of the right motors [-1.0..1.0]
   */
  public void tankDrive(double leftSpeed, double rightSpeed) {
    tankDrive(leftSpeed, rightSpeed, false);
  }

  public void tankDriveVolts(double leftSpeed, double rightSpeed) {
    leftMaster.set(TalonFXControlMode.PercentOutput,leftSpeed);
    rightMaster.set(TalonFXControlMode.PercentOutput, rightSpeed);
  }

  /**
   * Drives the robot by individually addressing the left and right side of the
   * drive train
   * 
   * @param leftSpeed  speed of the left motors [-1.0..1.0]
   * @param rightSpeed speed of the right motors [-1.0..1.0]
   * @param useSquares if set, decreases input sensitivity at low speeds
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean useSquares) {
    var xLeftSpeed = leftSpeed * MAX_SPEED_TELE;
    var xRightSpeed = rightSpeed * MAX_SPEED_TELE;
    if (useSquares) {
      xLeftSpeed *= Math.abs(xLeftSpeed);
      xRightSpeed *= Math.abs(xRightSpeed);
    }
    tankDriveVelocity(xLeftSpeed, xRightSpeed);
  }

  /**
   * Sets the neutral mode for the drive train
   * 
   * @param neutralMode the desired neutral mode
   */
  public void setNeutralMode(NeutralMode neutralMode) {
    leftMaster.setNeutralMode(neutralMode);
    leftSlave.setNeutralMode(neutralMode);
    rightMaster.setNeutralMode(neutralMode);
    rightSlave.setNeutralMode(neutralMode);
  }

  /**
   * returns left encoder position
   * 
   * @return left encoder position
   */
  public int getLeftEncoderPosition() {
    return leftMaster.getSelectedSensorPosition(0);
  }

  public double getLeftEncoderDistance() {
    return stepsToMeters(getLeftEncoderPosition());
  }

  /**
   * returns right encoder position
   * 
   * @return right encoder position
   */
  public int getRightEncoderPosition() {
    return rightMaster.getSelectedSensorPosition(0);
  }
  public double getRightEncoderDistance() {
    return stepsToMeters(getRightEncoderPosition());
  }
  

  private void zeroDriveTrainEncoders() {
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public void zeroSensors() {
    zeroDriveTrainEncoders();
    gyro.zeroYaw();
  }

  /**
   * Returns the heading of the robot in form required for odometry.
   *
   * @return the robot's heading in degrees, from 180 to 180 with positive value
   *         for left turn.
   */
  public double getHeading() {
    return Math.IEEEremainder(gyro.getAngle(), 360.0d) * -1.0d;
  }
  public double getYawRate() {
    return gyro.getRate();
  }

  /**
   * Controls the left and right side of the drive using Talon SRX closed-loop
   * velocity.
   * 
   * @param leftVelocity  left velocity
   * @param rightVelocity right velocity
   */
  public void tankDriveVelocity(double leftVelocity, double rightVelocity) {
    var leftAccel = (leftVelocity - stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity())) / 5;
    var rightAccel = (rightVelocity - stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity())) / 5;
    
    var leftFeedForwardVolts = FEED_FORWARD.calculate(leftVelocity, leftAccel);
    var rightFeedForwardVolts = FEED_FORWARD.calculate(rightVelocity, rightAccel);

    leftMaster.set(
        TalonFXControlMode.Velocity, 
        metersPerSecToStepsPerDecisec(leftVelocity), 
        DemandType.ArbitraryFeedForward,
        leftFeedForwardVolts / 12);
    rightMaster.set(
        TalonFXControlMode.Velocity,
        metersPerSecToStepsPerDecisec(rightVelocity),
        DemandType.ArbitraryFeedForward,
        rightFeedForwardVolts / 12);
  }

  /**
   * Converts from encoder steps to meters.
   * 
   * @param steps encoder steps to convert
   * @return meters
   */
  public static double stepsToMeters(int steps) {
    return (WHEEL_CIRCUMFERENCE_METERS / SENSOR_UNITS_PER_ROTATION) * steps;
  }

  /**
   * Converts from encoder units per 100 milliseconds to meters per second.
   * @param stepsPerDecisec steps per decisecond
   * @return meters per second
   */
  public static double stepsPerDecisecToMetersPerSec(int stepsPerDecisec) {
    return stepsToMeters(stepsPerDecisec * 10);
  }

  /**
   * Converts from meters to encoder units.
   * @param meters meters
   * @return encoder units
   */
  public static double metersToSteps(double meters) {
    return (meters / WHEEL_CIRCUMFERENCE_METERS) * SENSOR_UNITS_PER_ROTATION;
  }

  /**
   * Convers from meters per second to encoder units per 100 milliseconds.
   * @param metersPerSec meters per second
   * @return encoder units per decisecond
   */
  public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
    return metersToSteps(metersPerSec) * .1d;
  }

  public void calibrateGyro() {
    gyro.reset();
  }
}