/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  SynchronousPid controller = new SynchronousPid(0.06, 0, 0, 0);  
  TalonSRX motor = new TalonSRX(30);
  Joystick joy = new Joystick(0);
  double lastPos = 0;
  NetworkTable chameleon;
  NetworkTableEntry yaw;

  private static final int deviceID = 10;
  private CANSparkMax m_motorL, m_motorR;
  private CANPIDController m_pidControllerL, m_pidControllerR;
  private CANEncoder m_encoderL, m_encoderR;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;

  double ratio = 0.5d;

  ShotGenerator shotGenerator;
  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    controller.setOutputRange(1, -1);
    controller.setSetpoint(0);
    motor.enableVoltageCompensation(true);
    motor.configVoltageCompSaturation(12, 10);
    chameleon = NetworkTableInstance.getDefault().getTable("chameleon-vision");

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
    SmartDashboard.putNumber("Ratio Driven", 0);
    SmartDashboard.putNumber("Top Wheel", 0);
    m_motorR.setIdleMode(IdleMode.kCoast);
    m_motorL.setIdleMode(IdleMode.kCoast);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use
   * this for items like diagnostics that you want ran during disabled,
   * autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before
   * LiveWindow and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }


  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable
   * chooser code works with the Java SmartDashboard. If you prefer the
   * LabVIEW Dashboard, remove all of the chooser code and uncomment the
   * getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to
   * the switch structure below with additional strings. If using the
   * SendableChooser make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
    if(joy.getRawButton(4)){
      controller.setSetpoint(0);
      motor.set(ControlMode.PercentOutput, controller.update(getYaw()));
    }
    //controller.setSetpoint(0);
    //motor.set(ControlMode.PercentOutput, controller.update(getYaw()));
    double topSpeed = SmartDashboard.getNumber("Top Wheel", 0);
    
    if (topSpeed == 0) {
      m_motorL.set(0);
      m_motorR.set(0);
      return;
    }
    double botSpeed = topSpeed*4;
    m_pidControllerL.setReference(topSpeed, ControlType.kVelocity);
    m_pidControllerR.setReference(-botSpeed, ControlType.kVelocity);
    
    SmartDashboard.putNumber("EncoderL", m_encoderL.getVelocity());
    SmartDashboard.putNumber("EncoderR", m_encoderR.getVelocity());
    SmartDashboard.putNumber("TopSpeed", topSpeed);
    SmartDashboard.putNumber("BotSpeed", botSpeed);
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }

  public double getSensorDegrees(){

    double sensorPos = ((motor.getSensorCollection().getPulseWidthRiseToFallUs() - 1024) / 8f) / 4095f * 360;
    if (Math.abs(sensorPos - lastPos) > 100) {
      sensorPos = lastPos;
    }
    else
    lastPos = sensorPos;

    SmartDashboard.putNumber("sensorPos", sensorPos);

    return sensorPos;
  }

  public double getYaw(){
    yaw = chameleon.getSubTable("USB Camera-B4.09.24.1").getEntry("yaw");
    if(chameleon.getSubTable("USB Camera-B4.09.24.1").getEntry("is_valid").getBoolean(false)){
      return yaw.getDouble(0);
    }else{
      return 0;
    }
  }
}
