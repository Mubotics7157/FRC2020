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
public class SetTurretFieldLock extends AutoCommand{
    
    public void start() {
        Turret.getInstance().setFieldLock();
    }

    public boolean isFinished() {
        return true;
    }
}
