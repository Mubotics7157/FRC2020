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
public class SetTurretOff extends AutoCommand{
    public SetTurretOff(){
        this.setBlocking(false);
    }
    public void start() {
        Turret.getInstance().setOff();
    }

    public boolean isFinished() {
        return true;
    }
}
