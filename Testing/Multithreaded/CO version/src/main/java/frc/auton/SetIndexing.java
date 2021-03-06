package frc.auton;

import frc.subsystem.Drive;
import frc.subsystem.Indexer;

/**
 * Add your docs here.
 */
public class SetIndexing extends AutoCommand{
    private boolean index;
    public SetIndexing(boolean index) {
        this.setBlocking(false);
        this.index = index;
    }
    @Override
    public void start() {
        Indexer.getInstance().sideChew();
    }

    @Override
    public boolean isFinished() {
        return Drive.getInstance().isFinished();
    }
}
