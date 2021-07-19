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
    private double bot;
    private double top;
    private int times;
    public SetShooting(double b, double t, int times, boolean blocking) {
        bot = b;
        this.setBlocking(blocking);
        top = t;
        this.times = times;
    }
    @Override
    public void start() {
        Indexer.getInstance().setLemons(times);
        Indexer.getInstance().setShooting(bot, top);
    }

    @Override
    public boolean isFinished() {
        return Indexer.getInstance().getLemonCount() == 0;
    }
}
