/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.Constants.TurretConstants;
import frc.utility.LidarLitePWM;
import frc.utility.Threaded;
import frc.utility.VisionTarget;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LidarConstants;

public class Turret extends Threaded {
  /**
   * Creates a new Turret.
   */
  private boolean isHoming = false;
  private TalonSRX turretMotor = new TalonSRX(TurretConstants.DEVICE_ID_TURRET);
  private double fieldRelativeSetpoint;
  private double realSetpoint;
  private double lastRealSetpoint = 0;
  private double driveTrainHeading;
  private double lastFieldRelativeSetpoint = 0;
  private int smoothing = 0;
  private int pov = -1;
  private VisionManager vision;
  //public SynchronousPid turretPID = new SynchronousPid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, 0);
  
  private static final Turret trackingInstance = new Turret();
  
  private enum TurretState{
    OFF, //kys
    FIELD_LOCK, //field relative
    TARGET_LOCK, //vision
  }

  TurretState turretState = TurretState.OFF;
  LidarLitePWM lidar = new LidarLitePWM(new DigitalInput(LidarConstants.DIO_PORT));

	public static Turret getInstance() {
		return Turret.trackingInstance;
  }
  
  public Turret() {
    //turretPID.setOutputRange(1, -1);
    //turretPID.setSetpoint(0);
    turretMotor.configFactoryDefault();

		/* Configure Sensor Source for Primary PID */
		turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, TurretConstants.kPIDLoopIdx,
				TurretConstants.kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		turretMotor.configNeutralDeadband(0.001, TurretConstants.kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sesnor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		turretMotor.setSensorPhase(false);
		turretMotor.setInverted(false);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, TurretConstants.kTimeoutMs);
		turretMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, TurretConstants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		turretMotor.configNominalOutputForward(0, TurretConstants.kTimeoutMs);
		turretMotor.configNominalOutputReverse(0, TurretConstants.kTimeoutMs);
		turretMotor.configPeakOutputForward(1, TurretConstants.kTimeoutMs);
		turretMotor.configPeakOutputReverse(-1, TurretConstants.kTimeoutMs);

		/* Set Motion Magic gains in slot0 - see documentation */
		turretMotor.selectProfileSlot(TurretConstants.kSlotIdx, TurretConstants.kPIDLoopIdx);
		turretMotor.config_kF(TurretConstants.kSlotIdx, TurretConstants.kF, TurretConstants.kTimeoutMs);
		turretMotor.config_kP(TurretConstants.kSlotIdx, TurretConstants.kP, TurretConstants.kTimeoutMs);
		turretMotor.config_kI(TurretConstants.kSlotIdx, TurretConstants.kI, TurretConstants.kTimeoutMs);
		turretMotor.config_kD(TurretConstants.kSlotIdx, TurretConstants.kD, TurretConstants.kTimeoutMs);

		/* Set acceleration and vcruise velocity - see documentation */
		turretMotor.configMotionCruiseVelocity(15000, TurretConstants.kTimeoutMs);
		turretMotor.configMotionAcceleration(6000, TurretConstants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
    turretMotor.setSelectedSensorPosition(0, TurretConstants.kPIDLoopIdx, TurretConstants.kTimeoutMs);
    
    vision = VisionManager.getInstance();
  }

  @Override
  public void update() {

    driveTrainHeading = Drive.getInstance().getHeading();
    
    switch(turretState){
      case OFF:
        break;
      case FIELD_LOCK:
        updateFieldLock();
        break;
      case TARGET_LOCK:
        realSetpoint = vision.getTarget().getYaw() + driveTrainHeading;
        break;
    }

    lastRealSetpoint = realSetpoint;
    lastFieldRelativeSetpoint = fieldRelativeSetpoint;

    /* 4096 ticks/rev * realSetpoint(degrees) / 360 */
    if(turretState != TurretState.OFF){
      double targetPos = realSetpoint * 4096 / 360.0f;
      turretMotor.set(ControlMode.MotionMagic, targetPos);
    }
		
    //turretPID.update(getTurretPositionDegrees());
  }

  private void updateFieldLock(){
    realSetpoint = lastFieldRelativeSetpoint - driveTrainHeading;
  }

  public void SetHoming(boolean homing) {
    isHoming = homing;
  }

  public double getFieldRelativeHeading() {
    return Math.IEEEremainder(getTurretHeading() + Drive.getInstance().getHeading(), 360);
  }

  public synchronized void setTargetLock(){
    turretState = TurretState.TARGET_LOCK;
  }

  public synchronized void setOff(){
    turretState = TurretState.OFF;
  }

  public synchronized void setFieldLock(){
    turretState = TurretState.FIELD_LOCK;
  }

  public double getTurretHeading() {
    //between 0 and 4095
    return (turretMotor.getSensorCollection().getPulseWidthRiseToFallUs() - 1024) / 8f;
  }

  public double getAngleToInnerPort() {
    double distToPort = RobotTracker.getInstance().getDistance();
    double curTheta = VisionManager.getInstance().yaw.getDouble(0);
    return Math.atan(distToPort * Math.sin(curTheta) / (distToPort * Math.cos(curTheta)
       + FieldConstants.INTERPORT_METERS));
  }
}
