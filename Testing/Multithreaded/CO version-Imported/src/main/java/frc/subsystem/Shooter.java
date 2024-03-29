/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystem;
import com.revrobotics.CANPIDController;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ShooterConstants;
import frc.utility.LazyCANSparkMax;

/**
 * Add your docs here.
 */
public class Shooter {
    private LazyCANSparkMax botWheel;
    private LazyCANSparkMax topWheel;

    private double lastTop = 0;
    private double lastBot = 0;
    private double lastVelTop = 0;
    private double lastVelBot = 0;

    private double allowableRPMError = ShooterConstants.MAX_ALLOWABLE_ERROR_RPM;

    public Shooter() {
        botWheel = new LazyCANSparkMax(ShooterConstants.DEVICE_ID_SHOOTER_BOTTOM, MotorType.kBrushless, 11);
        topWheel = new LazyCANSparkMax(ShooterConstants.DEVICE_ID_SHOOTER_TOP, MotorType.kBrushless, 11);
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

    public synchronized void toggleRPMTolerance() {
        if(allowableRPMError == ShooterConstants.MAX_ALLOWABLE_ERROR_RPM) allowableRPMError = ShooterConstants.MAX_ALLOWABLE_ERROR_RPM_FART;
        else allowableRPMError = ShooterConstants.MAX_ALLOWABLE_ERROR_RPM;

        SmartDashboard.putNumber("allowable rpm error", allowableRPMError);
    }

    public synchronized double getRPMTolerance() {
        return allowableRPMError;
    }


    public boolean atSpeed(double bottom, double top) {
        if (bottom == 0 || top == 0) {
            topWheel.getPIDController().setReference(0, ControlType.kVoltage);
            botWheel.getPIDController().setReference(0, ControlType.kVoltage);
            return false;
        }
        lastTop = top;
        lastBot = bottom;
        topWheel.getPIDController().setReference(top, ControlType.kVelocity);
        botWheel.getPIDController().setReference(bottom, ControlType.kVelocity);
        SmartDashboard.putNumber("bottom Encoder", botWheel.getEncoder().getVelocity());
        SmartDashboard.putNumber("top Encoder", topWheel.getEncoder().getVelocity());
        SmartDashboard.putNumber("top accel", (topWheel.getEncoder().getVelocity() - lastVelTop) / .2f);
        SmartDashboard.putNumber("bot accel", (botWheel.getEncoder().getVelocity() - lastVelBot) / .2f);
        SmartDashboard.putNumber("top Setpoint", top);
        SmartDashboard.putNumber("bottom Setpoint", bottom);
        lastVelBot = botWheel.getEncoder().getVelocity();
        lastVelTop = topWheel.getEncoder().getVelocity();
        return 
            (Math.abs(topWheel.getEncoder().getVelocity() - top) < allowableRPMError) && 
            (Math.abs(botWheel.getEncoder().getVelocity() - bottom) < allowableRPMError);
    }

    public boolean lemonShot() { //ONLY USE WHEN ALSO USING ATSPEED
        SmartDashboard.putNumber("currBot", botWheel.getOutputCurrent());
        return 
            (Math.abs(topWheel.getEncoder().getVelocity() - lastTop) < ShooterConstants.LEMON_ERROR_COUNTER) &&
            (Math.abs(botWheel.getEncoder().getVelocity() - lastBot) < ShooterConstants.LEMON_ERROR_COUNTER);
    }

    public void setSpeed(double bottom, double top) {
        botWheel.set(bottom);
        topWheel.set(top);
    }
}
