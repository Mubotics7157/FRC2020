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

import frc.robot.Constants.TurretConstants;
import frc.utility.Threaded;
import frc.utility.VisionTarget;

public class Turret extends Threaded {
  /**
   * Creates a new Turret.
   */
  private boolean isHoming = false;
  private TalonSRX turretMotor = new TalonSRX(TurretConstants.DEVICE_ID_TURRET);
  private double fieldRelativeSetpoint;
  private double realSetpoint;
  private double driveTrainHeading;
  private double lastFieldRelativeSetpoint;
  private int smoothing = 0;
  private int pov = -1;
  private VisionManager vision = VisionManager.getInstance();
  //public SynchronousPid turretPID = new SynchronousPid(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, 0);

  
  private static final Turret trackingInstance = new Turret();
  

	public static Turret getInstance() {
		return Turret.trackingInstance;
  }
  
  public Turret() {
    //turretPID.setOutputRange(1, -1);
    //turretPID.setSetpoint(0);
    turretMotor.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		turretMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, TurretConstants.kPIDLoopIdx,
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

  }

  @Override
  public void update() {
    driveTrainHeading = Drive.getInstance().getHeading();
    fieldRelativeSetpoint = 0;

    if (isHoming){
      fieldRelativeSetpoint = vision.getTarget().getYaw();
      lastFieldRelativeSetpoint = fieldRelativeSetpoint;
    }else{
      fieldRelativeSetpoint = lastFieldRelativeSetpoint;
    }
    realSetpoint = fieldRelativeSetpoint - driveTrainHeading;    
    /* 4096 ticks/rev * realSetpoint(degrees) / 360 */
		double targetPos = realSetpoint * 4096 / 360.0f;
		turretMotor.set(ControlMode.MotionMagic, targetPos);
    //turretPID.update(getTurretPositionDegrees());
  }

  public void SetHoming(boolean homing) {
    isHoming = homing;
  }

  public double getTurretPosition() {
    //between 0 and 4095
    return (turretMotor.getSensorCollection().getPulseWidthRiseToFallUs() - 1024) / 8f;
  }

  public double getTurretPositionDegrees() {
    return (getTurretPosition() / 4095f) * 360f;
  }


}
