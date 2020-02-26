/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystem;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;

/**
 * Add your docs here.
 */
public class Shooter {
    private CANSparkMax botWheel;
    private CANSparkMax topWheel;

    public Shooter() {
        botWheel = new CANSparkMax(ShooterConstants.DEVICE_ID_SHOOTER_BOTTOM, MotorType.kBrushless);
        topWheel = new CANSparkMax(ShooterConstants.DEVICE_ID_SHOOTER_TOP, MotorType.kBrushless);
        CANPIDController botController = botWheel.getPIDController();
        CANPIDController topController = topWheel.getPIDController();

        botController.setP(ShooterConstants.kP);
        botController.setI(ShooterConstants.kI);
        botController.setD(ShooterConstants.kD);
        botController.setFF(ShooterConstants.kFF);
        
        topController.setP(ShooterConstants.kP);
        topController.setI(ShooterConstants.kI);
        topController.setD(ShooterConstants.kD);
        botController.setFF(ShooterConstants.kFF);
    }

    public boolean atSpeed(double bottom, double top) {
        topWheel.getPIDController().setReference(top, ControlType.kVelocity);
        botWheel.getPIDController().setReference(bottom, ControlType.kVelocity);
        return 
            (Math.abs(topWheel.getEncoder().getVelocity() - top) < ShooterConstants.MAX_ALLOWABLE_ERROR_RPM) && 
            (Math.abs(botWheel.getEncoder().getVelocity() - bottom) < ShooterConstants.MAX_ALLOWABLE_ERROR_RPM);
    }

    public void setSpeed(double bottom, double top) {
        botWheel.set(bottom);
        topWheel.set(top);
    }
}
