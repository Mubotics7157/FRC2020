// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;
import frc.utility.Threaded;
import frc.utility.VisionTarget;
public class VisionManager extends Threaded {

	private static final VisionManager trackingInstance = new VisionManager();

	public static VisionManager getInstance() {
		return VisionManager.trackingInstance;
    }
    
    private VisionTarget lastTarget;
	private VisionManager() {
	}
	/**
	 * Integrates the encoders and gyro to figure out robot position. A constant
	 * curvature is assumed
	 */
	@Override
	public void update() {
    }
    
    public VisionTarget getLastTarget() {
        return lastTarget;
    }
}