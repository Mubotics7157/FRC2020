package frc.auton;

import frc.subsystem.Drive;
import frc.subsystem.Indexer;

/**
 * Add your docs here.
 */
public class RunChute extends AutoCommand{
    private boolean swallow;
    private boolean runChute;
    public RunChute(boolean swallow, boolean runChute) {
        this.setBlocking(false);
        this.runChute = runChute;
        this.swallow = swallow;
    }
    @Override
    public void start() {
        //Indexer.getInstance().setIndexing(runChute);
        Indexer.getInstance().sideChew();
    }

    @Override
    public boolean isFinished() {
        return Drive.getInstance().isFinished();
    }
}
