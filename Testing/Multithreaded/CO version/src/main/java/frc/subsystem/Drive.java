package frc.subsystem;

import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_LEFT_MASTER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_LEFT_SHIFTER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_RIGHT_SHIFTER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_LEFT_SLAVE;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_RIGHT_MASTER;
import static frc.robot.Constants.DriveTrainConstants.DEVICE_ID_RIGHT_SLAVE;
import static frc.robot.Constants.DriveTrainConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.DriveTrainConstants.FEED_FORWARD;
import static frc.robot.Constants.TeleConstants.MAX_ANGULAR_VEL;
import static frc.robot.Constants.TeleConstants.MAX_SPEED_TELE;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.auton.PathTrigger;
import frc.robot.Robot;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;
import frc.utility.control.SynchronousPid;

/**
 * DriveTrainSubsystem
 */
public class Drive extends Threaded{

  public final TalonFX leftMaster = new TalonFX(DEVICE_ID_LEFT_MASTER);
  private final TalonFX leftSlave = new TalonFX(DEVICE_ID_LEFT_SLAVE);
  public final TalonFX rightMaster = new TalonFX(DEVICE_ID_RIGHT_MASTER);
  private final TalonFX rightSlave = new TalonFX(DEVICE_ID_RIGHT_SLAVE);
  private final Servo leftShifter = new Servo(DEVICE_ID_LEFT_SHIFTER);
  private final Servo rightShifter = new Servo(DEVICE_ID_RIGHT_SHIFTER);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  private RamseteController ramseteController = new RamseteController();
  private double ramsetePrevTime = 0;
  private final Timer ramseteTimer;
  private SynchronousPid turnPID = new SynchronousPid(0.01, 0, 0, 0);
  
	private static final Drive instance = new Drive(); 

	public static Drive getInstance() {
		return instance;
  }
  
	public enum DriveState {
		TELEOP, PUREPURSUIT, TURN, HOLD, DONE,TUNING
  }
  
  DriveState driveState = DriveState.TELEOP;
  Trajectory currentTrajectory;
  Rotation2d wantedHeading;
  private ArrayList<PathTrigger> triggers = new ArrayList<>();

  public Drive() {
    ramseteTimer = new Timer();
    zeroDriveTrainEncoders();
    gyro.zeroYaw();

    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor;
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
    
    leftMaster.setInverted(true);
    leftSlave.setInverted(true);
    rightMaster.setSensorPhase(true);
    leftMaster.setSensorPhase(true);
    rightMaster.overrideLimitSwitchesEnable(false);
    leftMaster.overrideLimitSwitchesEnable(false);

    leftSlave.follow(leftMaster);
    rightSlave.follow(rightMaster);
    gyro.reset();
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
          SmartDashboard.putString("Drive State", "Teleop");
        break;
        case PUREPURSUIT:          
        SmartDashboard.putString("Drive State", "Pure Pursuit");
        updatePathController();
          break;
        case TURN:
          updateTurn();
          break;
        case HOLD:
          updateHold();
          break;
        case DONE:
        SmartDashboard.putString("Drive State", "Done");
          break;

        case TUNING:
        SmartDashboard.putString("Drive State", "Tuning");
        updateTuning();
      }
  }

  public synchronized void setBreakMode(){
    leftMaster.setNeutralMode(NeutralMode.Brake);
    rightMaster.setNeutralMode(NeutralMode.Brake);
    rightSlave.setNeutralMode(NeutralMode.Brake);
   leftSlave.setNeutralMode(NeutralMode.Brake);
  }

  public synchronized void setAutoPath(Trajectory path, ArrayList<PathTrigger> triggers) {
    ramseteTimer.reset();
    ramseteTimer.start();
    currentTrajectory = path;
    driveState = DriveState.PUREPURSUIT;
    updatePathController();
    this.triggers = triggers;
  }

  public synchronized void setAutoPath(Trajectory path) {
    ramseteTimer.reset();
    ramseteTimer.start();
    currentTrajectory = path;
    driveState = DriveState.PUREPURSUIT;
    updatePathController();
  }
  
	public void setRotation(Rotation2d angle) {
		synchronized (this) {
			wantedHeading = angle;
			driveState = DriveState.TURN;
		}
  }   

  private double getPathPercentage() {
    SmartDashboard.putNumber("ramsete timer", ramseteTimer.get());
    SmartDashboard.putNumber("traj tim", currentTrajectory.getTotalTimeSeconds());
    return ramseteTimer.get() / currentTrajectory.getTotalTimeSeconds();
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

  public void setTuning(){
    synchronized(this){
      driveState = DriveState.TUNING;
    }
  }

  private void updateTuning() {
tankDriveVelocity(1, 1);

  }

  private void updateTeleOp() {

    driveTeleOp(Robot.leftStick.getRawAxis(1), Robot.rightStick.getRawAxis(1));
    SmartDashboard.putNumber("leftStick", Robot.rightStick.getRawAxis(1));
  }

  public void driveTeleOp(double l, double r) {
    double leftIn = 0;
    double rightIn = 0;
    if(Math.abs(l) >= DriveTrainConstants.DEADBAND)
      leftIn = l;
    if(Math.abs(r) >= DriveTrainConstants.DEADBAND)
      rightIn = r;
    tankDrive(leftIn, rightIn, false);
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
    double curTime = ramseteTimer.get();

    //play commands we pass
    while (!triggers.isEmpty()) {
			if (triggers.get(0).getPercentage() <= getPathPercentage()) {
        triggers.remove(0).playTrigger();
			} else {
				break;
			}
		}
    SmartDashboard.putNumber("path tim", getPathPercentage());
    double dt = (curTime - ramsetePrevTime) * 10;
    Trajectory.State goal = currentTrajectory.sample(curTime);
    ChassisSpeeds adjustedSpeeds = ramseteController.calculate(RobotTracker.getInstance().getOdometry(), goal);
    DifferentialDriveWheelSpeeds wheelSpeeds = DRIVE_KINEMATICS.toWheelSpeeds(adjustedSpeeds);
    double left = wheelSpeeds.leftMetersPerSecond;
    double right = wheelSpeeds.rightMetersPerSecond;
    boolean isFinished = ramseteTimer.hasPeriodPassed(currentTrajectory.getTotalTimeSeconds());
    if (isFinished) {
      ramseteTimer.stop();
      synchronized (this) {
        driveState = DriveState.DONE;
      }
    }
    tankDriveVelocity(left, right, dt);
    ramsetePrevTime = curTime;
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
    SmartDashboard.putNumber("left setpoint", leftSpeed);
    if (useSquares) {
      xLeftSpeed = Math.copySign(Math.pow(Math.abs(xLeftSpeed), 4), xLeftSpeed);
      xRightSpeed = Math.copySign(Math.pow(Math.abs(xRightSpeed), 4), xRightSpeed);
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

  public void shift(int gear) {
    leftShifter.set(gear);
    rightShifter.set(gear);
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
    SmartDashboard.putNumber("L Vel", leftVelocity);
    SmartDashboard.putNumber("R Vel", rightVelocity);

    var actualLeftVel = stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity());
    var actualRightVel = stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity());

    SmartDashboard.putNumber("Actual L Vel", actualLeftVel);
    SmartDashboard.putNumber("Actual R Vel", actualRightVel);
    SmartDashboard.putNumber("errorL", actualLeftVel - leftVelocity);
    SmartDashboard.putNumber("errorR", actualRightVel - rightVelocity);

    var leftAccel = (leftVelocity - actualLeftVel) / .20;
    var rightAccel = (rightVelocity - actualRightVel) / .20;
/*
    if (Math.abs(leftAccel) > TrajectoryConstants.MAX_ACCELERATION_AUTO) {
      leftAccel = Math.copySign(TrajectoryConstants.MAX_ACCELERATION_AUTO, leftAccel);
    }
    
    if (Math.abs(rightAccel) > TrajectoryConstants.MAX_ACCELERATION_AUTO) {
      rightAccel = Math.copySign(TrajectoryConstants.MAX_ACCELERATION_AUTO, rightAccel);
    }
    */
    var leftFeedForwardVolts = FEED_FORWARD.calculate(leftVelocity, leftAccel);
    var rightFeedForwardVolts = FEED_FORWARD.calculate(rightVelocity, rightAccel);

    if(leftVelocity == 0){
      leftMaster.set(TalonFXControlMode.PercentOutput, 0);
    }else{
      leftMaster.set(
        TalonFXControlMode.Velocity, 
        metersPerSecToStepsPerDecisec(leftVelocity), 
        DemandType.ArbitraryFeedForward,
        leftFeedForwardVolts / 12);
    }
    if(rightVelocity == 0){
      rightMaster.set(TalonFXControlMode.PercentOutput, 0);
    }else{
      rightMaster.set(
        TalonFXControlMode.Velocity,
        metersPerSecToStepsPerDecisec(rightVelocity),
        DemandType.ArbitraryFeedForward,
        rightFeedForwardVolts / 12);
    }

    
    
  }

  public void tankDriveVelocity(double leftVelocity, double rightVelocity, double dt) {
    SmartDashboard.putNumber("L Vel", leftVelocity);
    SmartDashboard.putNumber("R Vel", rightVelocity);

    var actualLeftVel = stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity());
    var actualRightVel = stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity());

    SmartDashboard.putNumber("Actual L Vel", actualLeftVel);
    SmartDashboard.putNumber("Actual R Vel", actualRightVel);
    SmartDashboard.putNumber("errorL", actualLeftVel - leftVelocity);
    SmartDashboard.putNumber("errorR", actualRightVel - rightVelocity);

    var leftAccel = (leftVelocity - actualLeftVel) / dt;
    var rightAccel = (rightVelocity - actualRightVel) / dt;
    
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
    double encoderConstant =
        (0.1524 * Math.PI) / 199.0879514239766 / 100;
    return steps * encoderConstant;
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
    return (meters / 0.1524 / Math.PI) * 100 * 199.0879514239766;
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