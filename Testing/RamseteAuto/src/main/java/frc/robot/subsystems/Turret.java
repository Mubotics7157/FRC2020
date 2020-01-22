/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.utility.control.SynchronousPID;

import static frc.robot.Constants.TurretConstants;

public class Turret extends SubsystemBase {
  /**
   * Creates a new Turret.
   */
  private boolean isHoming = false;
  private TalonSRX turretMotor = new TalonSRX(TurretConstants.DEVICE_ID_TURRET);
  public SynchronousPID turretPID = new SynchronousPID(TurretConstants.kP, TurretConstants.kI, TurretConstants.kD, 0);

  public Turret() {
    turretMotor.configSelectedFeedbackSensor(RemoteFeedbackDevice.SoftwareEmulatedSensor);
    turretPID.setOutputRange(1, -1);
    turretPID.setSetpoint(0);
  }

  @Override
  public void periodic() {

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
