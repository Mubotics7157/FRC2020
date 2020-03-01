/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystem;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;

/**
 * Add your docs here.
 */
public class Shooter {
    private CANSparkMax botWheel;
    private CANSparkMax topWheel;

    private double lastTop = 0;
    private double lastBot = 0;

    public Shooter() {
        botWheel = new CANSparkMax(ShooterConstants.DEVICE_ID_SHOOTER_BOTTOM, MotorType.kBrushless);
        topWheel = new CANSparkMax(ShooterConstants.DEVICE_ID_SHOOTER_TOP, MotorType.kBrushless);
        CANPIDController botController = botWheel.getPIDController();
        CANPIDController topController = topWheel.getPIDController();

        botController.setP(ShooterConstants.kP_BOTTOM);
        botController.setI(ShooterConstants.kI_BOTTOM);
        botController.setD(ShooterConstants.kD_BOTTOM);
        botController.setFF(ShooterConstants.kFF_BOTTOM);
        
        topController.setP(ShooterConstants.kP_TOP);
        topController.setI(ShooterConstants.kI_TOP);
        topController.setD(ShooterConstants.kD_TOP);
        topController.setFF(ShooterConstants.kFF_TOP);
    }

    public boolean atSpeed(double bottom, double top) {
        lastTop = top;
        lastBot = bottom;
        topWheel.getPIDController().setReference(top, ControlType.kVelocity);
        botWheel.getPIDController().setReference(bottom, ControlType.kVelocity);
        SmartDashboard.putNumber("bottom Encoder", botWheel.getEncoder().getVelocity());
        SmartDashboard.putNumber("top Encoder", topWheel.getEncoder().getVelocity());
        SmartDashboard.putNumber("top Setpoint", top);
        SmartDashboard.putNumber("bottom Setpoint", bottom);
        

        return 
            (Math.abs(topWheel.getEncoder().getVelocity() - top) < ShooterConstants.MAX_ALLOWABLE_ERROR_RPM) && 
            (Math.abs(botWheel.getEncoder().getVelocity() - bottom) < ShooterConstants.MAX_ALLOWABLE_ERROR_RPM);
    }

    public boolean lemonShot() { //ONLY USE WHEN ALSO USING ATSPEED
        return 
            (Math.abs(topWheel.getEncoder().getVelocity() - lastTop) < ShooterConstants.LEMON_ERROR_COUNTER) &&
            (Math.abs(botWheel.getEncoder().getVelocity() - lastBot) < ShooterConstants.LEMON_ERROR_COUNTER);
    }

    public void setSpeed(double bottom, double top) {
        botWheel.set(bottom);
        topWheel.set(top);
    }
}
