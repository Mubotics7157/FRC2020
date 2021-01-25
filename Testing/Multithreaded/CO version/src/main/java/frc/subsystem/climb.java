/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimbConstants;
import frc.utility.Threaded;


public class climb extends Threaded {
public TalonFX leftClimb;
  public TalonFX rightClimb; 
  public ClimbState climbState;
  double winchSpins;
  TalonFXConfiguration config = new TalonFXConfiguration();
  private double encoderValue;
  
  
  
  public climb(){
      climbState = ClimbState.STATIC;
      leftClimb = new TalonFX(ClimbConstants.DEVICE_ID_LEFT_CLIMB);//add to constants
      rightClimb = new TalonFX(ClimbConstants.DEVICE_ID_RIGHT_CLIMB);//add to constatns
      //leftClimb.enableVoltageCompensation(true);
      //rightClimb.enableVoltageCompensation(true);
      leftClimb.setNeutralMode(NeutralMode.Brake);
      rightClimb.setNeutralMode(NeutralMode.Brake);
      config.slot0.kP= ClimbConstants.kP;
      config.slot0.kD = ClimbConstants.kD;
      config.slot0.kF = ClimbConstants.kFF;
      leftClimb.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
      rightClimb.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, 10);
      leftClimb.setSensorPhase(true);
      rightClimb.setSensorPhase(true);
      zeroEncoders();
      rightClimb.follow(leftClimb);
      winchSpins = 32334234.345; //idk yet
      
  }
  
  public enum ClimbState{
    STATIC,
    EXTENDING,
    WINCHING,
    MANUAL
}

  public synchronized void setClimbState(ClimbState climbstate){
      this.climbState = climbstate;
      
  }
  
  public void zeroEncoders(){
    leftClimb.setSelectedSensorPosition(0);
    rightClimb.setSelectedSensorPosition(0);
  }

  public synchronized void updateEncoders(){
   encoderValue = getEncoderValue(leftClimb);
  }
  public double getEncoderValue(TalonFX falcon){
    return  falcon.getSelectedSensorPosition(0) / 128 / ClimbConstants.GEAR_DIVISOR;
  }

  public synchronized void Extend(double speed){   
      speed = speed> ClimbConstants.MAX_CLIMB_SPEED ? ClimbConstants.MAX_CLIMB_SPEED : speed; //change to normalize from orange utility
          if(getEncoderValue(leftClimb) == winchSpins && getEncoderValue(rightClimb) == winchSpins){
          leftClimb.set(ControlMode.Disabled, 0);
          setClimbState(ClimbState.STATIC);
          }
          else
          leftClimb.set(ControlMode.PercentOutput,speed);

  }

  public synchronized void Winch(double speed){
    zeroEncoders();
    speed = speed> ClimbConstants.MAX_CLIMB_SPEED ? ClimbConstants.MAX_CLIMB_SPEED * -1 : speed; //change to normalize from orange utility
    if(getEncoderValue(leftClimb) == winchSpins *-1 && getEncoderValue(rightClimb) == winchSpins * -1){
      leftClimb.set(ControlMode.Disabled, 0);
      setClimbState(ClimbState.STATIC);
      }
      else
      leftClimb.set(ControlMode.PercentOutput, speed);
  }


  public void findVoltageSetpoint(){
    SmartDashboard.putNumber("voltage", leftClimb.getMotorOutputVoltage());
    SmartDashboard.putNumber("voltage", rightClimb.getMotorOutputVoltage());
  }



  @Override
  public void update() {
    updateEncoders();

    switch(climbState){
        case STATIC:
            SmartDashboard.putString("climb state", "STATIC");
            break; 
        case EXTENDING:
            SmartDashboard.putString("climb state", "EXTENDING");
            Extend(.7);
            break;

        case WINCHING:
            Winch(-.7);
            SmartDashboard.putString("climb state", "WINCHING ");
          
          case MANUAL:
            //manualClimb(leftSpeed, rightSpeed);
            break;
    }
  
  }
}