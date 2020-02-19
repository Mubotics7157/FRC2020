/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystem;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;
import frc.utility.Threaded;

/**
 * Add your docs here.
 */
public class Shooter {
    private static final Shooter instance = new Shooter();
    private CANSparkMax botWheel;
    private CANSparkMax topWheel;
    public static Shooter getInstance() {
        return instance;
    }

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

    public void setSpeed(double bottom, double top) {
        topWheel.getPIDController().setReference(top, ControlType.kVelocity);
        botWheel.getPIDController().setReference(bottom, ControlType.kVelocity);
    }
}
