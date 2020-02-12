/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  NetworkTable chameleon;
  NetworkTableEntry yaw;
  @Override
  public void robotInit() {
    chameleon = NetworkTableInstance.getDefault().getTable("chameleon-vision");
    SmartDashboard.putNumber("distToPort", 0);
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  public double getYaw(){
    yaw = chameleon.getSubTable("USB Camera-B4.09.24.1").getEntry("yaw");
    if(chameleon.getSubTable("USB Camera-B4.09.24.1").getEntry("is_valid").getBoolean(false)){
      return yaw.getDouble(0);
    }else{
      return 0;
    }
  }
  public double getAngleToInnerPort() {
    double distToPort = SmartDashboard.getNumber("distToPort", 0);
    double curTheta = yaw.getDouble(0);
    return Math.atan(distToPort * Math.sin(curTheta) / (distToPort * Math.cos(curTheta) + 0.734));
  }

}
