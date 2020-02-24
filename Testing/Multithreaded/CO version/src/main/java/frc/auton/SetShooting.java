/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.auton;

import frc.subsystem.Indexer;

/**
 * Add your docs here.
 */
public class SetShooting extends AutoCommand{
    @Override
    public void start() {
        Indexer.getInstance().setShooting();
        this.setBlocking(false);
    }

    @Override
    public boolean isFinished() {
        return Indexer.getInstance().getLemonCount() == 0;
    }
}
