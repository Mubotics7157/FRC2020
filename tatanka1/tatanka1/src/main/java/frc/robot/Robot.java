/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends IterativeRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  public TalonSRX bottomLeft;
  public TalonSRX bottomRight;
  public TalonSRX frontRight;
  public TalonSRX frontLeft;
  public DoubleSolenoid intake;
  public DoubleSolenoid shooter;
  public DoubleSolenoid ramp;
  public Joystick controller = new Joystick(0);
  public VictorSPX intakeLeft;
  public VictorSPX intakeRight;
  public VictorSPX shooterBottomLeft;
  public VictorSPX shooterBottomRight;
  public TalonSRX shooterUpperLeft;
  public TalonSRX shooterUpperRight;

  /**
   * This function is run when the robot is first started up and should be
   * used for any initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    bottomLeft = new TalonSRX(11);
    bottomRight = new TalonSRX(10);
    frontLeft = new TalonSRX(20);
    frontRight = new TalonSRX(21);
    intakeLeft = new VictorSPX(30);
    intakeRight = new VictorSPX(31);
    shooterBottomLeft = new VictorSPX(40);
    shooterBottomRight = new VictorSPX(41);
    shooterUpperLeft = new TalonSRX(42);
    shooterUpperRight = new TalonSRX(43);

    intake = new DoubleSolenoid(0, 1);
    shooter = new DoubleSolenoid(2, 3);
    ramp = new DoubleSolenoid(4, 5);
    bottomLeft.follow(frontLeft);
    bottomRight.follow(frontRight);
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
    // autoSelected = SmartDashboard.getString("Auto Selector",
    // defaultAuto);
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
    frontLeft.set(ControlMode.PercentOutput, controller.getRawAxis(1));
    frontRight.set(ControlMode.PercentOutput, controller.getRawAxis(5));

    if (controller.getRawButtonPressed(1)){
      shooter.set(Value.kForward);
    }

    if(controller.getRawButtonPressed(2)){
      ramp.set(Value.kForward);
    }

    if(controller.getRawButtonPressed(3)){
      shooter.set(Value.kReverse);
    }

    if(controller.getRawButtonPressed(4))
    {
      shooter.set(Value.kReverse);
    }

    if(controller.getRawButton(10)){
      setShooter(1);
    }else{
      setShooter(0);
    }

    if(controller.getRawAxis(2)>0){
      setIntake(controller.getRawAxis(2));
      intake.set(Value.kForward);
    }else{
      setIntake(0);
      intake.set(Value.kReverse);
    }

    if(controller.getRawButton(11)){
      ramp.set(Value.kForward);
    }else{
      ramp.set(Value.kReverse);
    }
 

  }

  public void setShooter(double speed){
    shooterBottomRight.set(ControlMode.PercentOutput, speed);
    shooterBottomLeft.set(ControlMode.PercentOutput, -speed);
    shooterUpperRight.set(ControlMode.PercentOutput, speed);
    shooterUpperLeft.set(ControlMode.PercentOutput, -speed);
  }

  public void setIntake(double speed){
    intakeLeft.set(ControlMode.PercentOutput, speed);
    intakeRight.set(ControlMode.PercentOutput, -speed);
  }

  




  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
