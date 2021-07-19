/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.auton;

import frc.subsystem.Turret;

/**
 * Add your docs here.
 */
public class SetTurretAngle extends AutoCommand{

    private double _setpoint = 0;

    public SetTurretAngle(double setpoint){
        _setpoint = setpoint;
        this.setBlocking(false);
    }

    public void start() {
        //Turret.getInstance().setTargetLock();
        Turret.getInstance().setArbitraryLock();
        Turret.getInstance().setRealSetpoint(_setpoint);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
