/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.auton;

import frc.subsystem.Drive;
import frc.subsystem.Indexer;

/**
 * Add your docs here.
 */
public class SetIntaking extends AutoCommand{
    private boolean hungry;
    private boolean down;
    public SetIntaking(boolean hungry, boolean down) {
        this.hungry = hungry;
        this.down = down;
    }
    @Override
    public void start() {
        Indexer.getInstance().setHungry(hungry);
        Indexer.getInstance().setSalivation(down);
        this.setBlocking(false);
    }

    @Override
    public boolean isFinished() {
        return Drive.getInstance().isFinished();
    }
}
