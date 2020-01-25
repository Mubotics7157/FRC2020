package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * This is a demo program showing the use of the RobotDrive class, specifically
 * it contains the code necessary to operate a robot with tank drive.
 */
public class Robot extends TimedRobot {
  private Joystick m_stick;
  private static final int deviceID = 10;
  private CANSparkMax m_motorL, m_motorR;
  private CANPIDController m_pidControllerL, m_pidControllerR;
  private CANEncoder m_encoderL, m_encoderR;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private double speed = 0;

  @Override
  public void robotInit() {
    m_stick = new Joystick(0);

    // initialize motor
    m_motorL = new CANSparkMax(deviceID, MotorType.kBrushless);
    m_motorR = new CANSparkMax(deviceID+1, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motorL.restoreFactoryDefaults();
    m_motorR.restoreFactoryDefaults();

    /**
     * In order to use PID functionality for a controller, a CANPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidControllerL = m_motorL.getPIDController();
    m_pidControllerR = m_motorR.getPIDController();

    // Encoder object created to display position values
    m_encoderL = m_motorL.getEncoder();
    m_encoderR = m_motorR.getEncoder();

    // PID coefficients
    kP = 0.00045; 
    kI = 0.0000002;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    // set PID coefficients
    m_pidControllerL.setP(kP);
    m_pidControllerL.setI(kI);
    m_pidControllerL.setD(kD);
    m_pidControllerL.setIZone(kIz);
    m_pidControllerL.setFF(kFF);
    m_pidControllerL.setOutputRange(kMinOutput, kMaxOutput);

    
    // set PID coefficients
    m_pidControllerR.setP(kP);
    m_pidControllerR.setI(kI);
    m_pidControllerR.setD(kD);
    m_pidControllerR.setIZone(kIz);
    m_pidControllerR.setFF(kFF);
    m_pidControllerR.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Top Wheel", 0);
    SmartDashboard.putNumber("Bot Wheel", 0);
    m_motorR.setIdleMode(IdleMode.kCoast);
    m_motorL.setIdleMode(IdleMode.kCoast);

  }

  @Override
  public void teleopPeriodic() {
    /*if (true) {

      if (m_stick.getRawButtonPressed(11)) {
        speed += 0.1;
      }

      if (m_stick.getRawButtonPressed(12)) {
        speed -= 0.1;
      }

      if (m_stick.getRawButtonPressed(13)) {
        speed = 0;
      }
      m_motorL.set(-speed);
      m_motorR.set(speed*0.5);
      return;
    }*/
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidControllerL.setP(p); kP = p; }
    if((i != kI)) { m_pidControllerL.setI(i); m_pidControllerR.setI(i);kI = i; }
    if((d != kD)) { m_pidControllerL.setD(d); kD = d; }
    if((iz != kIz)) { m_pidControllerL.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidControllerL.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidControllerL.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    double topSpeed = SmartDashboard.getNumber("Top Wheel", 0);
    double botSpeed = SmartDashboard.getNumber("Bot Wheel", 0);
    SmartDashboard.putNumber("Backspin Ratio", topSpeed / botSpeed);

    if (botSpeed != 0) {
    m_pidControllerL.setReference(-botSpeed, ControlType.kVelocity);
    }
    else {
      m_motorL.set(0);
    }

    if (topSpeed != 0) {
    m_pidControllerR.setReference(topSpeed, ControlType.kVelocity);
    }
    else {
      m_motorR.set(0);
    }

    SmartDashboard.putNumber("ProcessVariable", m_encoderL.getVelocity());
    SmartDashboard.putNumber("ProcessVariable2", m_encoderR.getVelocity());
  }
}