// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import frc.robot.Constants;
import frc.utility.LazyTalonSRX;
import frc.utility.OrangeUtility;
import frc.utility.Threaded;
import frc.utility.control.RateLimiter;
import frc.utility.control.SynchronousPid;
import frc.utility.control.motion.Path;
import frc.utility.control.motion.PurePursuitController;
import frc.utility.math.Rotation2D;

import java.io.FileWriter;
import java.io.IOException;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utility.LazyCANSparkMax;

public class Drive extends Threaded {
	//#region Helper structures
	public enum DriveState {
		TELEOP, PUREPURSUIT, TURN, HOLD, DONE
	}
	public static class DriveSignal {
		/*
		 * Inches per second for speed
		 */
		public double leftVelocity;
		public double rightVelocity;
		public double leftAcc;
		public double rightAcc;

		public DriveSignal(double left, double right) {
			this(left, 0, right, 0);
		}

		public DriveSignal(double left, double leftAcc, double right, double rightAcc) {
			leftVelocity = left;
			this.leftAcc = leftAcc;
			rightVelocity = right;
			this.rightAcc = rightAcc;
		}
	}

	public static class AutoDriveSignal {
		public DriveSignal command;
		public boolean isDone;

		public AutoDriveSignal(DriveSignal command, boolean isDone) {
			this.command = command;
			this.isDone = isDone;
		}
	}
	//#endregion
	//#region Variable Galore
	private static final Drive instance = new Drive();

	public static Drive getInstance() {
		return instance;
	}

	private double quickStopAccumulator;
	private boolean drivePercentVbus;

	private AHRS gyroSensor;
	private PurePursuitController autonomousDriver;
	private SynchronousPid turnPID;
	private DriveState driveState;
	private RateLimiter moveProfiler, turnProfiler;
	private Solenoid shifter;
	private Rotation2D wantedHeading;
	private volatile double driveMultiplier;

	double prevPositionL = 0;
	double prevPositionR = 0;

	public LazyCANSparkMax leftSpark, rightSpark, leftSparkSlave, rightSparkSlave, leftSparkSlave2, rightSparkSlave2;
  	private CANPIDController leftSparkPID, rightSparkPID;
	private CANEncoder leftSparkEncoder, rightSparkEncoder;

	//used for logging velocity vs voltage
	private FileWriter leftWriter;
	private FileWriter rightWriter;
	private double leftVoltage = 0;
	private double rightVoltage = 0;
	//#endregion
	//#region init
	private Drive() {

		gyroSensor = new AHRS(Port.kMXP);

		leftSpark = new LazyCANSparkMax(Constants.DriveLeftMasterId, MotorType.kBrushless);
		leftSparkSlave = new LazyCANSparkMax(Constants.DriveLeftSlave1Id, MotorType.kBrushless);
		rightSpark = new LazyCANSparkMax(Constants.DriveRightMasterId, MotorType.kBrushless);
		rightSparkSlave = new LazyCANSparkMax(Constants.DriveRightSlave1Id, MotorType.kBrushless);

		leftSpark.setInverted(true);
		rightSpark.setInverted(false);
		leftSparkSlave.setInverted(true);
		rightSparkSlave.setInverted(false);

		leftSparkPID = leftSpark.getPIDController();
		rightSparkPID = rightSpark.getPIDController();
		leftSparkEncoder = leftSpark.getEncoder();
		rightSparkEncoder = rightSpark.getEncoder();

		configMotors();

		drivePercentVbus = true;
		driveState = DriveState.TELEOP;

		turnPID = new SynchronousPid(1.0, 0, 1.2, 0); //P=1.0 OR 0.8
		turnPID.setOutputRange(Constants.DriveSpeed, -Constants.DriveSpeed);
		turnPID.setSetpoint(0);

		moveProfiler = new RateLimiter(Constants.DriveTeleopAccLimit);
		turnProfiler = new RateLimiter(100);
		
		configAuto();
	}

	private void configAuto() {
		rightSparkPID.setP(Constants.kDriveRightAutoP, 0);
		rightSparkPID.setD(Constants.kDriveRightAutoD, 0);
		rightSparkPID.setFF(Constants.kRv,0);
		rightSparkPID.setOutputRange(-1, 1);


		leftSparkPID.setP(Constants.kDriveLeftAutoP, 0);
		leftSparkPID.setD(Constants.kDriveLeftAutoD, 0);
		leftSparkPID.setFF(Constants.kLv,0);
		leftSparkPID.setOutputRange(-1, 1);

		
		leftSpark.setIdleMode(IdleMode.kBrake);
		rightSpark.setIdleMode(IdleMode.kBrake);
		leftSparkSlave.setIdleMode(IdleMode.kBrake);
		rightSparkSlave.setIdleMode(IdleMode.kBrake);
	}

	public void configMotors() {
		leftSparkSlave.follow(leftSpark);
		rightSparkSlave.follow(rightSpark);
		
		leftSpark.setIdleMode(IdleMode.kCoast);
		rightSpark.setIdleMode(IdleMode.kCoast);
		leftSparkSlave.setIdleMode(IdleMode.kCoast);
		rightSparkSlave.setIdleMode(IdleMode.kCoast);

		
		leftSparkEncoder.setPositionConversionFactor(Constants.DriveConversionFactor);
		rightSparkEncoder.setPositionConversionFactor(Constants.DriveConversionFactor);
		leftSparkEncoder.setVelocityConversionFactor(Constants.VelConversionFactor);
		rightSparkEncoder.setVelocityConversionFactor(Constants.VelConversionFactor);
	}

	@Override
	public void update() {
	DriveState snapDriveState;
		synchronized (this) {
			snapDriveState = driveState;
		}
		switch (snapDriveState) {
			case TELEOP:
				break;
			case PUREPURSUIT:
				//System.out.println("bad!");
				updatePurePursuit();
				break;
			case TURN:
				updateTurn();
				break;
			case HOLD:
				hold();
				break;
		}
		
	}
	//#endregion
	//#region DEBUG
	public void debug() {
		System.out.println("L enc: " + getLeftDistance()+ " velo " + getLeftSpeed()); 
		System.out.println("R enc: " + getRightDistance() + " velo " + getRightSpeed()); 
		System.out.println("Gyro: " + getAngle()/*getGyroAngle().getDegrees()*/);
	}

	public void debugSpeed() {
		SmartDashboard.putNumber("Left Speed", getLeftSpeed());
		SmartDashboard.putNumber("Right Speed", getRightSpeed());
	}

	
	public void debugDistance() {
		SmartDashboard.putNumber("Left Distance", getLeftDistance());
		SmartDashboard.putNumber("Right Distance", getRightDistance());
	}

	public void debugVoltage() {
		SmartDashboard.putNumber("Avg Voltage", getVoltage());
	}

	public void printCurrent() {
		System.out.println(leftSpark);
	}

	public void debugDriveFF(double lSpeed, double rSpeed) {
		setWheelVelocity(new DriveSignal(lSpeed, rSpeed));
	}
	//#endregion
    //#region Drive Methods
    public void skidLimitingDrive(double moveValue, double rotateValue) {
		synchronized(this) {
			driveState = DriveState.TELEOP;
		}
		moveValue = scaleJoystickValues(moveValue, 0);
		rotateValue = scaleJoystickValues(rotateValue, 0);
        
		double leftMotorSpeed;
		double rightMotorSpeed;
		// Square values but keep sign
		moveValue = Math.copySign(Math.pow(moveValue, 2), moveValue);
		rotateValue = Math.copySign(Math.pow(rotateValue, 2), rotateValue);
        
        //Linear
        /*
        double slope = -1.25;
        double maxRotate = slope * Math.abs(moveValue) + 1;
        */
        //Nonlinear       
        double curvature = 5;
        double curveCenter = 0.5;
       
        
        //Concave up
        //y = (0.5 / (5 * x)) - (0.5 / 5)
        //double maxRotate = curveCenter / (curvature * Math.abs(moveValue)) - (curveCenter / curvature);
        
        //Concave down
        //y = -2^(5 * (x - 1)) + 1
        double maxRotate = -Math.pow((1 / curveCenter), curvature * (Math.abs(moveValue) - 1)) + 1;
        
        rotateValue = OrangeUtility.coerce(rotateValue, maxRotate, -maxRotate);
        
		double maxValue = Math.abs(moveValue) + Math.abs(rotateValue);
		if (maxValue > 1) {
			moveValue -= Math.copySign(maxValue - 1, moveValue);
		}

		leftMotorSpeed = moveValue + rotateValue;
		rightMotorSpeed = moveValue - rotateValue;
		if (drivePercentVbus) {
			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		} else {
			leftMotorSpeed = moveValue + rotateValue;
			rightMotorSpeed = moveValue - rotateValue;
			leftMotorSpeed *= Constants.DriveSpeed;
			rightMotorSpeed *= Constants.DriveSpeed;
			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
	}
	
	public void arcadeDrive(double moveValue, double rotateValue) {
		//String toPrint="";
		//double time = Timer.getFPGATimestamp();
		synchronized(this) {
			driveState = DriveState.TELEOP;
		}
		moveValue = scaleJoystickValues(moveValue, 0);
		rotateValue = scaleJoystickValues(rotateValue, Constants.steeringWheel ? 1 : 0);
		//double t = Timer.getFPGATimestamp() - time;
		//if(teleopstart) toPrint += (t) + " 12\n";

		double leftMotorSpeed;
		double rightMotorSpeed;
		// Square values but keep sign
		moveValue = Math.copySign(Math.pow(moveValue, 2), moveValue);
		rotateValue = Math.copySign(Math.pow(rotateValue, 2), rotateValue);
		double maxValue = Math.abs(moveValue) + Math.abs(rotateValue);
		if (maxValue > 1) {
			moveValue -= Math.copySign(maxValue - 1, moveValue);
		}
		//if(teleopstart) toPrint += (Timer.getFPGATimestamp() - time) + " 12\n";

		leftMotorSpeed = moveValue + rotateValue;
		rightMotorSpeed = moveValue - rotateValue;
		if (drivePercentVbus) {
			//System.out.println("left " + getLeftSpeed() + " power: " + leftMotorSpeed);
			//System.out.println("right " + getRightSpeed() + " power: " + rightMotorSpeed);

			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		} else {
			leftMotorSpeed = moveValue + rotateValue*0.5;
			rightMotorSpeed = moveValue - rotateValue*0.5;
			leftMotorSpeed *= Constants.DriveSpeed;
			rightMotorSpeed *= Constants.DriveSpeed;
			
			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
	}

	public void cheesyDrive(double moveValue, double rotateValue, boolean isQuickTurn) {
		synchronized (this) {
			driveState = DriveState.TELEOP;
		}
		moveValue = scaleJoystickValues(moveValue, 0);
		rotateValue = scaleJoystickValues(rotateValue, Constants.steeringWheel ? 1 : 0);

		double leftMotorSpeed;
		double rightMotorSpeed;
		double angularPower = 1;

		double overPower;

		if (isQuickTurn) {
			overPower = 1;
			if (moveValue < 0.2) {
				quickStopAccumulator = 0.9 * quickStopAccumulator + 0.1 * rotateValue * 2;
			}
			angularPower = rotateValue * 0.4; //0.2
		} else {
			overPower = 0;
			angularPower = Math.abs(moveValue) * rotateValue - quickStopAccumulator;
			if (quickStopAccumulator > 1) {
				quickStopAccumulator -= 1;
			} else if (quickStopAccumulator < -1) {
				quickStopAccumulator += 1;
			} else {
				quickStopAccumulator = 0;
			}
		}

		// moveValue = moveProfiler.update(moveValue * driveMultiplier) /
		// driveMultiplier;
		leftMotorSpeed = moveValue + angularPower;
		rightMotorSpeed = moveValue - angularPower;

		if (leftMotorSpeed > 1.0) {
			rightMotorSpeed -= overPower * (leftMotorSpeed - 1.0);
			leftMotorSpeed = 1.0;
		} else if (rightMotorSpeed > 1.0) {
			leftMotorSpeed -= overPower * (rightMotorSpeed - 1.0);
			rightMotorSpeed = 1.0;
		} else if (leftMotorSpeed < -1.0) {
			rightMotorSpeed += overPower * (-1.0 - leftMotorSpeed);
			leftMotorSpeed = -1.0;
		} else if (rightMotorSpeed < -1.0) {
			leftMotorSpeed += overPower * (-1.0 - rightMotorSpeed);
			rightMotorSpeed = -1.0;
		}

		leftMotorSpeed = OrangeUtility.coerce(leftMotorSpeed, 1, -1);
		rightMotorSpeed = OrangeUtility.coerce(rightMotorSpeed, 1, -1);	

		if (drivePercentVbus) {
			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		} else {
			leftMotorSpeed *= driveMultiplier;
			rightMotorSpeed *= driveMultiplier;
			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
	}
	public void orangeDrive(double moveValue, double rotateValue, boolean isQuickTurn) {
		synchronized (this) {
			driveState = DriveState.TELEOP;
		}
		moveValue = scaleJoystickValues(moveValue, 0);
		rotateValue = scaleJoystickValues(rotateValue, Constants.steeringWheel ? 1 : 0);
		// 50 is min turn radius
		double radius = (1 / rotateValue) + Math.copySign(24, rotateValue);
		double deltaSpeed = (Constants.TrackRadius * ((moveValue * driveMultiplier) / radius));
		deltaSpeed /= driveMultiplier;
		if (isQuickTurn) {
			deltaSpeed = rotateValue;
		}
		double leftMotorSpeed = moveValue + deltaSpeed;
		double rightMotorSpeed = moveValue - deltaSpeed;
		if (leftMotorSpeed > 1.0) {
			rightMotorSpeed -= (leftMotorSpeed - 1.0);
			leftMotorSpeed = 1.0;
		} else if (rightMotorSpeed > 1.0) {
			leftMotorSpeed -= (rightMotorSpeed - 1.0);
			rightMotorSpeed = 1.0;
		} else if (leftMotorSpeed < -1.0) {
			rightMotorSpeed += (-1.0 - leftMotorSpeed);
			leftMotorSpeed = -1.0;
		} else if (rightMotorSpeed < -1.0) {
			leftMotorSpeed += (-1.0 - rightMotorSpeed);
			rightMotorSpeed = -1.0;
		}
		if (drivePercentVbus) {
			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		} else {
			leftMotorSpeed *= driveMultiplier;
			rightMotorSpeed *= driveMultiplier;
			if (leftMotorSpeed == 0 && rightMotorSpeed == 0) {
				setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
			}
			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
	}

	public void tankDrive(double leftValue, double rightValue, boolean vbus) {
		synchronized (this) {
			driveState = DriveState.TELEOP;
		}
		leftValue = scaleJoystickValues(leftValue, 0);
		rightValue = scaleJoystickValues(rightValue, 0);

		double leftMotorSpeed = leftValue;
		double rightMotorSpeed = rightValue;
		if (vbus) {
			setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		} else {
			leftMotorSpeed *= driveMultiplier;
			rightMotorSpeed *= driveMultiplier;
			if (leftMotorSpeed == 0 && rightMotorSpeed == 0) {
				setWheelPower(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
			}
			setWheelVelocity(new DriveSignal(leftMotorSpeed, rightMotorSpeed));
		}
	}

	public void jankDrive(double left, double right){
		synchronized(this){
			driveState = DriveState.TELEOP;
		}
		setWheelPower(new DriveSignal(left, right));
	}
	//#endregion
	//#region Public
	synchronized public void setTeleop() {
		driveState = DriveState.TELEOP;
	}
	public void calibrateGyro() {
		gyroSensor.reset();
	}

	public void startHold() {
		prevPositionL = getLeftDistance();
		prevPositionR = getRightDistance();
		driveState = DriveState.HOLD;
	}

	public void endHold() {
		driveState = DriveState.TELEOP;
	}

	public void hold() {
		double errorL = prevPositionL - getLeftDistance();
		double errorR = prevPositionR - getRightDistance();
		setWheelVelocity(new DriveSignal(errorL * Constants.kHoldP , errorR* Constants.kHoldP));
	}

	
	public void resetMotionProfile() {
		moveProfiler.reset();
	}

	public double getAngle() {
		return gyroSensor.getAngle();
	}

	public double getDistance() {
		return (getLeftDistance() + getRightDistance()) / 2;
	}

	public Rotation2D getGyroAngle() {
		// -180 through 180
		return Rotation2D.fromDegrees(gyroSensor.getAngle());
	}

	public double getLeftDistance() {
		return leftSparkEncoder.getPosition();
	}

	public double getRightDistance() {
		return rightSparkEncoder.getPosition();
	}

	public double getSpeed() {
		return (getLeftSpeed()+getRightSpeed())/2;
	}

	public double getLeftSpeed() {
		return leftSparkEncoder.getVelocity();
	}

	public double getRightSpeed() {
		return rightSparkEncoder.getVelocity();
	}

	public double scaleJoystickValues(double rawValue, int profile) {
		return Math.copySign(OrangeUtility.coercedNormalize(Math.abs(rawValue), Constants.MinControllerInput[profile],
				Constants.MaxControllerInput, Constants.MinControllerOutput, Constants.MaxControllerOutput),
				rawValue);
	}

	public synchronized void setAutoPath(Path autoPath, boolean isReversed) {
		driveState = DriveState.PUREPURSUIT;
		autonomousDriver = new PurePursuitController(autoPath, isReversed);
		autonomousDriver.resetTime();
		configAuto();
		updatePurePursuit();
	}

	public double getVoltage() {
		return (Math.abs(leftSpark.getAppliedOutput()) + Math.abs(rightSpark.getAppliedOutput())) / 2;
	}

	private void setWheelPower(DriveSignal setVelocity) {
		leftSpark.set(setVelocity.leftVelocity);
		rightSpark.set(setVelocity.rightVelocity);
	}

	private void setWheelVelocity(DriveSignal setVelocity) {
		System.out.println("poopy");
		if (Math.abs(setVelocity.rightVelocity) > Constants.DriveSpeed) {
			DriverStation.getInstance();
			DriverStation.reportError("Velocity set over " + Constants.DriveSpeed + " !", false);
			
			return;
		}
		//setpoints = desired voltage
		double leftSetpoint = setVelocity.leftVelocity;
		double rightSetpoint = setVelocity.rightVelocity;

		SmartDashboard.putNumber("leftSetpoint", leftSetpoint);
		SmartDashboard.putNumber("rightSetpoint", rightSetpoint);

		leftSparkPID.setReference(leftSetpoint + Math.copySign((Constants.kLVi + Constants.kLa)/Constants.kLv, setVelocity.leftVelocity), ControlType.kVelocity, 0);
		rightSparkPID.setReference(rightSetpoint + Math.copySign((Constants.kRVi + Constants.kRa)/Constants.kRv, setVelocity.rightVelocity), ControlType.kVelocity, 0);
	}

	public synchronized void setSimpleDrive(boolean setting) {
		drivePercentVbus = setting;
	}

	public void setRotation(Rotation2D angle) {
		synchronized (this) {
			wantedHeading = angle;
			driveState = DriveState.TURN;
		}
	}
	
	synchronized public boolean isFinished() {
		return driveState == DriveState.DONE || driveState == DriveState.TELEOP;
	}
	//#endregion
	//#region FF Graphing
	public void getFFGraph() {
		System.out.println("ffgraph");
		try {
		leftWriter = new FileWriter("/home/lvuser/left.csv");
		rightWriter = new FileWriter("/home/lvuser/right.csv");
		System.out.println("inside loop");

		leftWriter.append("Voltage");
		leftWriter.append(",");
		leftWriter.append("Velocity");
		leftWriter.append("\n");

		rightWriter.append("Voltage");
		rightWriter.append(",");
		rightWriter.append("Velocity");
		rightWriter.append("\n");
		}
		catch (IOException e) {
			System.out.println(e);
		}
	}

	public void writeFFPeriodic() {
		leftVoltage+=0.25/12*0.02;
		rightVoltage+=0.25/12*0.02;
		setWheelPower(new DriveSignal(leftVoltage, rightVoltage));

		try {
		leftWriter.append(Double.toString(leftVoltage));
		leftWriter.append(",");
    	leftWriter.append(Double.toString(leftSparkEncoder.getVelocity()));
		leftWriter.append("\n");

		rightWriter.append(Double.toString(leftVoltage));
		rightWriter.append(",");
    	rightWriter.append(Double.toString(rightSparkEncoder.getVelocity()));
		rightWriter.append("\n");
		}
		catch (IOException e) {

		}
	}

	public void FFClose() {
		setWheelPower(new DriveSignal(0,0));
		System.out.println("------------------------------");
		try {
		leftWriter.flush();
		leftWriter.close();

		rightWriter.flush();
		rightWriter.close();
		}
		catch (IOException e) {

		}
		leftVoltage=0;
		rightVoltage=0;
	}
	//#endregion
	//#region Motion Control
	private void updateTurn() {
		double error = wantedHeading.rotateBy(RobotTracker.getInstance().getOdometry().rotationMat.inverse()).getDegrees();
		double deltaSpeed;
		//System.out.println(RobotTracker.getInstance().getOdometry().rotationMat.getDegrees());
		//System.out.println("error: " + error);
		deltaSpeed = turnPID.update(error);
		deltaSpeed = Math.copySign(
				OrangeUtility.coercedNormalize(Math.abs(deltaSpeed), 0, 180, 0, Constants.DriveSpeed), deltaSpeed);
		if (Math.abs(error) < Constants.maxTurnError && deltaSpeed < Constants.maxPIDStopSpeed) {
			setWheelVelocity(new DriveSignal(0, 0));
			synchronized (this) {
				driveState = DriveState.DONE;
			} 
		} else {
			setWheelVelocity(new DriveSignal(-deltaSpeed, deltaSpeed));
		}
	}

	private void updatePurePursuit() {
		AutoDriveSignal signal = autonomousDriver.calculate(RobotTracker.getInstance().getOdometry());
		if (signal.isDone) {
			synchronized (this) {
				driveState = DriveState.DONE;
			}
		}
		setWheelVelocity(signal.command);
	}

	synchronized public void stopMovement() {
		leftSpark.set(0);
		rightSpark.set(0);
		leftSparkPID.setReference(0, ControlType.kDutyCycle);
		rightSparkPID.setReference(0, ControlType.kDutyCycle);
		setWheelVelocity(new DriveSignal(0,0));

		driveState = DriveState.TELEOP;
		resetMotionProfile();
	}
	//#endregion
}
