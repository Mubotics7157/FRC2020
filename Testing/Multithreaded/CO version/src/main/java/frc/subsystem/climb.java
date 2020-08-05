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
  public ClimbState cState;
  
  public climb(){
      cState = ClimbState.STATIC;
      leftClimb = new TalonFX(ClimbConstants.DEVICE_ID_LEFT_CLIMB);//add to constants
      rightClimb = new TalonFX(ClimbConstants.DEVICE_ID_RIGHT_CLIMB);//add to constatns
      //leftClimb.enableVoltageCompensation(true);
      //rightClimb.enableVoltageCompensation(true);
      leftClimb.setNeutralMode(NeutralMode.Brake);
      rightClimb.setNeutralMode(NeutralMode.Brake);
      rightClimb.follow(leftClimb);

      
  }
  
  public double ticksToCM(double val){
    return val / 4096.0 /9.6666666; 
  }
  public synchronized void setclimbState(ClimbState climbstate){
      cState = climbstate;
      
  }
      
  public enum ClimbState{
      STATIC,
      EXTENDING,
      WINCHING
  }



  public void Extend(){   
          leftClimb.set(ControlMode.PercentOutput,.7);
  }

  public void Winch(){
      leftClimb.set(ControlMode.PercentOutput, -.7);
  }



  



  @Override
  public void update() {
    switch(cState){
        case STATIC:
            SmartDashboard.putString("climb", "STATIC");
            break; 
        case EXTENDING:
            SmartDashboard.putString("climb", "EXTENDING");
            Extend();
            break;

        case WINCHING:
            Winch();
            SmartDashboard.putString("climb", "WINCHING ");

    }
  
  }
}
