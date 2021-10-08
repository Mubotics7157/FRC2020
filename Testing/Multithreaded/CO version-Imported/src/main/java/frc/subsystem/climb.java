/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.ClimbConstants;
import frc.utility.Threaded;


public class climb extends Threaded {

  private TalonSRX climbMotor;
  private ClimbState climbState;
  private Solenoid lockSolenoid;
  private double realSetpoint = 0;
  private double targetPosition;
  private double manualSetpoint;
  private static climb instance = new climb();

  public static climb getInstance(){
    return instance;
  }

  @Override
  public void update() {
    ClimbState snapClimbState;
    synchronized(this){
      snapClimbState = climbState;
    }
    switch(snapClimbState){
      case EXTENDING:
        updateExtending();
        SmartDashboard.putString("climb state", "extending");
        break;
      
      case RETRACTING:
        updateRetracting();
        SmartDashboard.putString("climb state", "retracting");
        break;
      
      case STATIC:
        SmartDashboard.putString("climb state", "static");
        break;
      
      case MANUAL:
        SmartDashboard.putString("climb state", "manual");
        updateManual();
        break;

      case OFF:
        SmartDashboard.putString("climb state", "OFF");
        break;
      
    }

    targetPosition = realSetpoint *4096.0f; // do I need another conversion factor??? might be unsafe to test for now
    if(climbState != ClimbState.OFF || climbState != ClimbState.STATIC|| climbState!=ClimbState.MANUAL)
      climbMotor.set(ControlMode.MotionMagic, targetPosition);
    
    else if(climbState == ClimbState.STATIC)
      climbMotor.set(ControlMode.Position, climbMotor.getSelectedSensorPosition());

    SmartDashboard.putNumber("Climb Encoder stuff", climbMotor.getSelectedSensorPosition()/4096.0f);
  }

  public climb(){
    climbMotor = new TalonSRX(ClimbConstants.DEVICE_ID_LEFT_CLIMB);

    climbMotor.setSensorPhase(true);
    climbMotor.setInverted(false);

    climbMotor.selectProfileSlot(ClimbConstants.kSlotIdx, ClimbConstants.kPIDLoopx);
    climbMotor.config_kP(ClimbConstants.kSlotIdx, ClimbConstants.kP);
    climbMotor.config_kI(ClimbConstants.kSlotIdx, ClimbConstants.kI);
    climbMotor.config_kD(ClimbConstants.kSlotIdx, ClimbConstants.kD);
    climbMotor.config_kF(ClimbConstants.kSlotIdx, ClimbConstants.kF);

    climbMotor.setSelectedSensorPosition(0, ClimbConstants.kPIDLoopx, ClimbConstants.kTimeoutMs);

    climbMotor.configPeakOutputForward(1, ClimbConstants.kTimeoutMs);
    climbMotor.configPeakOutputReverse(1, ClimbConstants.kTimeoutMs);
    climbMotor.configNominalOutputForward(0,ClimbConstants.kTimeoutMs);
    climbMotor.configNominalOutputReverse(0,ClimbConstants.kTimeoutMs);

  }
  public enum ClimbState{
    OFF,
    EXTENDING,
    RETRACTING,
    STATIC,
    MANUAL
  }

  public synchronized void setExtending(){
    // realSetpoint = however tall climb mechanism should be
    climbState = ClimbState.EXTENDING;
  }

  public synchronized void setStatic(){
    climbState = ClimbState.STATIC;
  }

  public synchronized void setRetracting(){
    // realSetpoint = however tall climb mechanism should be
    toggleSolenoidLock();
    climbState = ClimbState.RETRACTING;
  }

  public synchronized void setManual(){
    climbState = ClimbState.MANUAL;
  }

  public synchronized void setOff(){
    climbState = ClimbState.OFF;
  }

  private void toggleSolenoidLock(){
    if(lockSolenoid.get())
      lockSolenoid.set(false);
    
    else
      lockSolenoid.set(true);

  }

  private void updateManual(){
    //realSetpoint = manualSetpoint;
    climbMotor.set(ControlMode.PercentOutput, Robot.xbox.getRawAxis(2));
  }

  public synchronized void adjustClimbPosition(int input){
    //manualSetpoint += input;
  }

  private void updateExtending(){

    if(getClimbPosition()==targetPosition){
      setStatic();

    }

  }

  private void updateRetracting(){

    if(getClimbPosition() == targetPosition){
      setStatic();
    }

  }

  public synchronized double getClimbPosition(){
    return climbMotor.getSelectedSensorPosition() /4096.0f; // is this the right conversion factor????
  }
}