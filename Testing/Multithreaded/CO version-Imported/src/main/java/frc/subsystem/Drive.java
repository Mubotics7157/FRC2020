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

public class Drive extends Threaded{

private final TalonFX leftMaster = new TalonFX(DEVICE_ID_LEFT_MASTER);
private final TalonFX leftSlave = new TalonFX(DEVICE_ID_LEFT_SLAVE);
private final TalonFX rightMaster = new TalonFX(DEVICE_ID_RIGHT_MASTER);
private final TalonFX rightSlave = new TalonFX(DEVICE_ID_RIGHT_MASTER);

private Servo leftShifter = new Servo(DEVICE_ID_LEFT_SHIFTER);
private Servo rightShifter = new Servo(DEVICE_ID_RIGHT_SHIFTER);

private AHRS gyro = new AHRS(SPI.Port.kMXP);

DriveState driveState = DriveState.TELEOP;

private Timer timer = new Timer();
private RamseteController ramseteController = new RamseteController();

private double previousTime;
private ArrayList <PathTrigger> triggers = new ArrayList<>();
Rotation2d wantedHeading;
Trajectory currentTrajectory;

private SynchronousPid turnPID = new SynchronousPid(0.01, 0, 0, 0);

private static final Drive instance = new Drive();

public Drive(){

  gyro.zeroYaw();

  leftMaster.setNeutralMode(NeutralMode.Coast);
  leftSlave.setNeutralMode(NeutralMode.Coast);
  rightMaster.setNeutralMode(NeutralMode.Coast);
  rightSlave.setNeutralMode(NeutralMode.Coast);

  leftMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,10);
  rightMaster.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor,0,10);

  leftMaster.setInverted(true);
  rightMaster.setInverted(false);
  leftSlave.setInverted(true);
  rightSlave.setInverted(false);
  
  rightMaster.setSensorPhase(true);
  leftMaster.setSensorPhase(true);
  leftSlave.setSensorPhase(true);
  rightSlave.setSensorPhase(true);
  leftMaster.overrideLimitSwitchesEnable(false);
  rightMaster.overrideLimitSwitchesEnable(false);
  leftMaster.configClosedloopRamp(100);
  rightMaster.configClosedloopRamp(100);
  rightSlave.configClosedloopRamp(100);
  leftSlave.configClosedloopRamp(100);
  leftMaster.configPeakOutputForward(.8);
  leftSlave.configPeakOutputForward(.8);
  rightMaster.configPeakOutputForward(.8);
  rightSlave.configPeakOutputForward(.8);

  leftSlave.follow(leftMaster);
  rightSlave.follow(rightMaster);

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
  gyro.reset();

}

  @Override
  public void update() {
    DriveState snapDriveState;
    
    synchronized(this){
      snapDriveState = driveState;
    }

    switch(snapDriveState){
      case TELEOP:
        updateTeleop();
        SmartDashboard.putString("Drive State", "Teleop");
        break;

      case PATHFOLLOWING:
        SmartDashboard.putString("Drive State", "Pathfollowing");
        updatePathController();
        break;

      case TURN:
        SmartDashboard.putString("Drive State", "Turning");
        updateTurn();
        break;
        
        case HOLD:
          updateHold();
          SmartDashboard.putString("Drive State", "Hold");
          break;

      case DONE:
        SmartDashboard.putString("Drive State", "Done");
        break;
    }
    debugDriveTrainMotors();
  }


  public enum DriveState{
    TELEOP,
    PATHFOLLOWING,
    TURN,
    HOLD,
    DONE
  }

  public void setTeleOp(){
    synchronized(this){
      driveState = DriveState.TELEOP;
    }
  }

  public void setHold(){
    synchronized(this){
      driveState = DriveState.HOLD;
    }
  }

  public static Drive getInstance(){
    return instance;
  }

  public synchronized void setAutoPath(Trajectory trajectory){
    timer.reset();
    timer.start();
    currentTrajectory = trajectory;
    driveState = DriveState.PATHFOLLOWING;
    updatePathController();
  }

    public synchronized void setAutoPath(Trajectory trajectory, ArrayList<PathTrigger> triggers){
    timer.reset();
    timer.start();
    currentTrajectory = trajectory;
    //ramsete tolerance
    driveState = DriveState.PATHFOLLOWING;
    updatePathController();
    this.triggers = triggers;
  }

  private double getPathPercentage(){
    return timer.get()/currentTrajectory.getTotalTimeSeconds();
  }


  private void updateTeleop(){
    tankDriveTeleop(Robot.leftStick.getRawAxis(1), Robot.rightStick.getRawAxis(1));
  }

  private void updatePathController(){
    double currentTime = timer.get();
    SmartDashboard.putBoolean("finished", isFinished());
        while (!triggers.isEmpty()) {
			if (triggers.get(0).getPercentage() <= getPathPercentage()) {
        triggers.remove(0).playTrigger();
			} else {
				break;
			}
		}
   
      Trajectory.State desiredPose = currentTrajectory.sample(currentTime);
      ChassisSpeeds speeds = ramseteController.calculate(RobotTracker.getInstance().getOdometry(), desiredPose);
      DifferentialDriveWheelSpeeds calculatedSpeeds = DRIVE_KINEMATICS.toWheelSpeeds(speeds);
      double leftSpeed = calculatedSpeeds.leftMetersPerSecond;
      double rightSpeed = calculatedSpeeds.rightMetersPerSecond;
      double timeChange = (currentTime-previousTime) *10 ;
      SmartDashboard.putNumber("desired poseX", desiredPose.poseMeters.getX());
      SmartDashboard.putNumber("desired poseY", desiredPose.poseMeters.getY());
      SmartDashboard.putNumber("desired angle", desiredPose.poseMeters.getRotation().getDegrees());
      boolean isFinished =  timer.hasPeriodPassed(currentTrajectory.getTotalTimeSeconds());


   if(isFinished){
     synchronized(this){
      driveState=DriveState.DONE;
     }
     timer.stop();
   }

   tankDriveVelocity(leftSpeed, rightSpeed, timeChange);
   previousTime = currentTime;
   //outputTelemtery();

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

  private void updateHold(){
    leftMaster.set(ControlMode.Position, leftMaster.getSelectedSensorPosition());
    rightMaster.set(ControlMode.Position, rightMaster.getSelectedSensorPosition());
  }

  public boolean isFinished(){
    return driveState ==DriveState.DONE;
  }

  	public void setRotation(Rotation2d angle) {
		synchronized (this) {
			wantedHeading = angle;
			driveState = DriveState.TURN;
		}
  }

  private void tankDrivePercentOutput(double leftSpeed, double rightSpeed){
    leftMaster.set(ControlMode.PercentOutput,leftSpeed);
    rightMaster.set(ControlMode.PercentOutput,rightSpeed);
  }

  private void tankDrive(double left,double right, boolean squaredInputs){
    double xleft = left*MAX_SPEED_TELE;
    double xright = right*MAX_SPEED_TELE;

    if(squaredInputs){
      xleft = Math.copySign(Math.pow(Math.abs(xleft),4), xleft);
      xright = Math.copySign(Math.pow(Math.abs(xright),4), xright);
    }

    SmartDashboard.putNumber("left Setpoint", left);
    SmartDashboard.putNumber("right Setpoint", right);
    tankDriveVelocity(xleft, xright);

  }

  private void tankDriveTeleop(double left, double right){
    double l = 0;
    double r = 0;
    if(Math.abs(left)>=DriveTrainConstants.DEADBAND)
      l=left;
      
    if(Math.abs(right)>=DriveTrainConstants.DEADBAND)
      r = right;
    
      tankDrive(l, r, false);
    
  }
 /**
   * sets the drive train to drive at a given velocity
   * to calculate the feedforward to get us to that velocity we need to know our acceleration and velocity setpoints
   * acceleration is modeled as a=Δv/Δt
   * velocity must be in SI units: meters/sec 
   * 
   * @param left left velocity of robot in meters/sec
   * @param right right velocity of robot in meters/sec
   */
  private void tankDriveVelocity(double left, double right){

    double actualLeftVelocity = stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity());
    double actualRightVelocity = stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity());
    SmartDashboard.putNumber("actual left vel", actualLeftVelocity);
    SmartDashboard.putNumber("actual right vel", actualRightVelocity);
    double leftAccel = (left-actualLeftVelocity)/.2; 
    double rightAccel = (right-actualRightVelocity)/.2; 
    double leftFeedForward = FEED_FORWARD.calculate(left, leftAccel);
    double rightFeedForward = FEED_FORWARD.calculate(right, rightAccel);
    if(left == 0)
      leftMaster.set(ControlMode.PercentOutput,0);
    
    else
      leftMaster.set(ControlMode.Velocity,metersPerSecToStepsPerDecisec(left),DemandType.ArbitraryFeedForward,leftFeedForward/12);

    if(right == 0)
      rightMaster.set(ControlMode.PercentOutput,0);

    else
      rightMaster.set(ControlMode.Velocity,metersPerSecToStepsPerDecisec(right),DemandType.ArbitraryFeedForward,rightFeedForward/12);

    
    

  }
 /**
   * sets the drive train to drive at a given velocity
   * to calculate the feedforward to get us to that velocity we need to know our acceleration and velocity setpoints
   * acceleration is modeled as a=Δv/Δt
   * velocity must be in SI units: meters/sec 
   * @param left left velocity of robot in meters/sec
   * @param right right velocity of robot in meters/sec
   * @param deltaTime change in time
   */
  private void tankDriveVelocity(double left, double right, double deltaTime){
      double actualLeftVelocity = stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity());
    double actualRightVelocity = stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity());
    SmartDashboard.putNumber("actual left vel", actualLeftVelocity);
    SmartDashboard.putNumber("actual right vel", actualRightVelocity);
    double leftAccel = (left-actualLeftVelocity)/deltaTime; 
    double rightAccel = (right-actualRightVelocity)/deltaTime; 
    double leftFeedForward = FEED_FORWARD.calculate(left, leftAccel);
    double rightFeedForward = FEED_FORWARD.calculate(right, rightAccel);
    if(left == 0)
      leftMaster.set(ControlMode.PercentOutput,0);
    
      else
        leftMaster.set(ControlMode.Velocity,metersPerSecToStepsPerDecisec(left),DemandType.ArbitraryFeedForward,leftFeedForward/12);

    if(right == 0)
      rightMaster.set(ControlMode.PercentOutput,0);

     else
       rightMaster.set(ControlMode.Velocity,metersPerSecToStepsPerDecisec(right),DemandType.ArbitraryFeedForward,rightFeedForward/12);

    
    
    
  }

  public void shift(int value){
    leftShifter.set(value);
    rightShifter.set(value);
  }

  public void setNeutralMode(NeutralMode mode){
    leftMaster.setNeutralMode(mode);
    rightMaster.setNeutralMode(mode);
    leftSlave.setNeutralMode(mode);
    rightSlave.setNeutralMode(mode);
  }

  private double getLeftEncoder(){
    return leftMaster.getSelectedSensorPosition(0);
  }

  private double getRightEncoder(){
    return rightMaster.getSelectedSensorPosition(0);
  }


  public double getLeftEncoderDistance(){
    return stepsToMeters(getLeftEncoder());
  }

  public double getRightEncoderDistance(){
    return stepsToMeters(getRightEncoder());
  } 

private void outputTelemtery(){
  SmartDashboard.putNumber("left distance", getLeftEncoderDistance());
  SmartDashboard.putNumber("right distance", getRightEncoderDistance());
  SmartDashboard.putNumber("angle", getHeading());
}

private void getCurrentDraw(){
  SmartDashboard.putNumber("left master current output",leftMaster.getStatorCurrent());
  SmartDashboard.putNumber("left slave current output",leftSlave.getStatorCurrent());
  SmartDashboard.putNumber("right master current output",leftMaster.getStatorCurrent());
  SmartDashboard.putNumber("right slave current output",rightSlave.getStatorCurrent());

}

private void getCurrentInput(){
  SmartDashboard.putNumber("left master current input",leftMaster.getSupplyCurrent());
  SmartDashboard.putNumber("left slave current input",leftSlave.getSupplyCurrent());
  SmartDashboard.putNumber("right master current input",leftMaster.getSupplyCurrent());
  SmartDashboard.putNumber("right slave current input",rightSlave.getSupplyCurrent());
}

private void getMotorTemp(){
  SmartDashboard.putNumber("left master temperature",leftMaster.getTemperature());
  SmartDashboard.putNumber("left slave temperature",leftSlave.getTemperature());
  SmartDashboard.putNumber("right master temperature",leftMaster.getTemperature());
  SmartDashboard.putNumber("right slave temperature",rightSlave.getTemperature());
}

private void getCanBusVoltage(){ // if they are the same you can delete one of the lines
  SmartDashboard.putNumber("Can Bus Voltage Left", leftMaster.getBusVoltage());
  SmartDashboard.putNumber("Can Bus Voltage Right", rightMaster.getBusVoltage());
}

private void getCanBusID(){
  SmartDashboard.putNumber("Left Slave ID", leftSlave.getDeviceID());
  SmartDashboard.putNumber("Left Master ID", leftMaster.getDeviceID());
}

private void debugDriveTrainMotors(){
  getCurrentDraw();
  getCurrentInput();
  getMotorTemp();
  getCanBusVoltage();
  getCanBusID();

} 

  

  public Rotation2d getDriveRotation2d(){
    return gyro.getRotation2d();
  }

  public synchronized DifferentialDriveWheelSpeeds getRates(){
    return new DifferentialDriveWheelSpeeds(stepsPerDecisecToMetersPerSec(leftMaster.getSelectedSensorVelocity()),stepsPerDecisecToMetersPerSec(rightMaster.getSelectedSensorVelocity()));
  }
  public double getHeading(){
    //might have to change the output
    //return gyro.getAngle();   
    return Math.IEEEremainder(gyro.getAngle(), 360.0d) * -1.0d; //why????
  }

  public void calibrateGyro(){
    gyro.reset();
  }

  private void resetEncoders(){
    leftMaster.setSelectedSensorPosition(0);
    rightMaster.setSelectedSensorPosition(0);
  }

  public void zeroSensors(){
    resetEncoders();
    gyro.zeroYaw();
  }




  private static double stepsToMeters(double steps){
    return steps*DriveTrainConstants.ENCODER_METER_CONSTANT;
  }


  /**
   * Converts from encoder units per 100 milliseconds to meters per second.
   * @param stepsPerDecisec steps per decisecond
   * @return meters per second
   */
  public static double stepsPerDecisecToMetersPerSec(double stepsPerDecisec) {
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
   * Converts from meters per second to encoder units per 100 milliseconds.
   * @param metersPerSec meters per second
   * @return encoder units per decisecond
   */
  public static double metersPerSecToStepsPerDecisec(double metersPerSec) {
    return metersToSteps(metersPerSec) * .1d;
  }

}