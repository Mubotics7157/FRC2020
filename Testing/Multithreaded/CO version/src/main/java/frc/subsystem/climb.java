/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimbConstants;
import frc.utility.Threaded;


public class climb extends Threaded {
public TalonFX leftClimb;
  public TalonFX rightClimb; 
  public ClimbState climbState;
  
  public climb(){
      climbState = ClimbState.STATIC;
      leftClimb = new TalonFX(ClimbConstants.DEVICE_ID_LEFT_CLIMB);//add to constants
      rightClimb = new TalonFX(ClimbConstants.DEVICE_ID_RIGHT_CLIMB);//add to constatns
      //leftClimb.enableVoltageCompensation(true);
      //rightClimb.enableVoltageCompensation(true);
      leftClimb.setNeutralMode(NeutralMode.Brake);
      rightClimb.setNeutralMode(NeutralMode.Brake);
      rightClimb.follow(leftClimb);
      
  }
  
  public enum ClimbState{
    STATIC,
    EXTENDING,
    WINCHING
}

  public double ticksToCM(double val){
    return val / 4096.0 /9.6666666; 
  }
  public synchronized void setclimbState(ClimbState climbstate){
      this.climbState = climbstate;
      
  }
      

  public void Extend(double speed){   
      speed = speed> ClimbConstants.MAX_CLIMB_SPEED ? ClimbConstants.MAX_CLIMB_SPEED : speed;
          leftClimb.set(ControlMode.PercentOutput,speed);
  }

  public void Winch(double speed){
    speed = speed> ClimbConstants.MAX_CLIMB_SPEED ? ClimbConstants.MAX_CLIMB_SPEED * -1 : speed;
      leftClimb.set(ControlMode.PercentOutput, speed);
  }



  @Override
  public void update() {
    switch(cState){
        case STATIC:
            SmartDashboard.putString("climb state", "STATIC");
            break; 
        case EXTENDING:
            SmartDashboard.putString("climb state", "EXTENDING");
            Extend();
            break;

        case WINCHING:
            Winch();
            SmartDashboard.putString("climb state", "WINCHING ");

    }
  
  }
}
