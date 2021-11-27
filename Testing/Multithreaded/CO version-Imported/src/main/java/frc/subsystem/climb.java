package frc.subsystem;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Constants.ClimbConstants;
import frc.utility.Threaded;


public class Climb extends Threaded {

  private TalonFX climbMotor;
  private ClimbState climbState;
  private double realSetpoint = 0;
  private double targetPosition;
  private static final Climb instance = new Climb();

  public static Climb getInstance(){
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
       //climbMotor.set(ControlMode.MotionMagic, targetPosition);
        break;
      
      case RETRACTING:
        updateRetracting();
        SmartDashboard.putString("climb state", "retracting");
     // climbMotor.set(ControlMode.MotionMagic, targetPosition);
        break;
      
      case MANUAL:
        SmartDashboard.putString("climb state", "manual");
        updateManual();
        break;

      case OFF:
        SmartDashboard.putString("climb state", "OFF");
        break;
      
    }

    targetPosition = realSetpoint *4096.0f; 
    //if(climbState == ClimbState.RETRACTING || climbState == ClimbState.EXTENDING)
     // climbMotor.set(ControlMode.MotionMagic, targetPosition);
    

    SmartDashboard.putNumber("Climb Position", climbMotor.getSelectedSensorPosition());
  }
  
  public Climb(){
    climbMotor = new TalonFX(ClimbConstants.DEVICE_ID_CLIMB);

    climbMotor.setSensorPhase(true);
    climbMotor.setInverted(true);

    climbMotor.selectProfileSlot(ClimbConstants.kSlotIdx, ClimbConstants.kPIDLoopx);
    climbMotor.config_kP(ClimbConstants.kSlotIdx, ClimbConstants.kP);
    climbMotor.config_kI(ClimbConstants.kSlotIdx, ClimbConstants.kI);
    climbMotor.config_kD(ClimbConstants.kSlotIdx, ClimbConstants.kD);
    climbMotor.config_kF(ClimbConstants.kSlotIdx, ClimbConstants.kF);


    climbMotor.setSelectedSensorPosition(0, ClimbConstants.kPIDLoopx, ClimbConstants.kTimeoutMs);

    climbMotor.configPeakOutputForward(1, ClimbConstants.kTimeoutMs);
    climbMotor.configPeakOutputReverse(-1, ClimbConstants.kTimeoutMs);
    climbMotor.configNominalOutputForward(0,ClimbConstants.kTimeoutMs);
    climbMotor.configNominalOutputReverse(0,ClimbConstants.kTimeoutMs);

    //climbMotor.configClosedloopRamp(.5);
    climbMotor.configReverseSoftLimitEnable(true);
    climbMotor.configReverseSoftLimitThreshold(-1); //-2048
    climbMotor.setNeutralMode(NeutralMode.Coast);

    climbMotor.setInverted(true);
    climbMotor.setSensorPhase(true);

  }

  public enum ClimbState{
    OFF,
    EXTENDING,
    RETRACTING,
    MANUAL
  }

  public synchronized void setExtending(){
    // realSetpoint = however tall climb mechanism should be
    climbState = ClimbState.EXTENDING;
  }


  public synchronized void setRetracting(){
    // realSetpoint = however tall climb mechanism should be
    climbState = ClimbState.RETRACTING;
  }

  public synchronized void setManual(){
    climbState = ClimbState.MANUAL;
  }

  public synchronized void setOff(){
    climbState = ClimbState.OFF;
  }

  private void updateManual(){
    //SmartDashboard.putNumber("Climb value", Robot.xbox.getRawAxis(1));
    //climbMotor.set(ControlMode.PercentOutput, Robot.xbox.getRawAxis(2));
  }


  private void updateExtending(){


  }

  private void updateRetracting(){


  }

  public synchronized double getClimbPosition(){
    return climbMotor.getSelectedSensorPosition() /4096.0f; // is this the right conversion factor????
  }

  public synchronized void setclimber(double speed){
    climbMotor.set(ControlMode.PercentOutput, speed);
  }
}