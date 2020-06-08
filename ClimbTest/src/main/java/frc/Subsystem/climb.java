        package frc.robot;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;                            
public class climb {
    public TalonFX leftClimb;
    public TalonFX rightClimb; 
    public climbState cState;
    
    //if we using hooks:
    public Solenoid hooks;
    
    public climb(){
        cState = climbState.OFF;
        leftClimb = new TalonFX(69);//add to constants
        rightClimb = new TalonFX(69);//add to constatns
        hooks = new Solenoid(69);//add to constants
        
        
    }

    public void setclimbState(climbState climbstate){
        cState = climbstate;
        
    }
        
    public enum climbState{
        OFF,
        EXTENDING,
        EXTENDED,
        WINCHING,
        DONE

    }

    public void doClimb(){
            switch(cState){
                case OFF:
                    SmartDashboard.putString("climb", "off");
                    break; //do nothing
                case EXTENDING:
                    SmartDashboard.putString("climb", "extending");
                    Extend();
                    break;
                case EXTENDED:
                    //checkExtended method using encoder
                    SmartDashboard.putString("climb", "extended");
                    deployHooks();
                    break;
                case WINCHING:
                    Winch();
                    SmartDashboard.putString("climb", "winching");
                case DONE:
                    SmartDashboard.putString("climb", "done");
            }
    }

    public void Extend(){   
            leftClimb.set(ControlMode.PercentOutput,.3);
            rightClimb.follow(leftClimb);
            //rightClimb.set(ControlMode.PercentOutput, .1);
    }

    public void Winch(){
        leftClimb.set(ControlMode.PercentOutput, -.3);
        rightClimb.follow(leftClimb);
    }

    public void deployHooks(){
        hooks.set(true);
        
    }

    public void retractHooks(){ //map this to a button in case something goes horribly wrong during testing
        hooks.set(false);
    }

    public boolean isExtended(){ //add encoder method
        
        return false;
    }

}