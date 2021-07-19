// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.utility.Threaded;
import frc.utility.VisionTarget;
import frc.robot.Constants.VisionConstants;
public class VisionManager extends Threaded {

	private static final VisionManager trackingInstance = new VisionManager();
	//NetworkTable chameleon;
	NetworkTable photon;
	NetworkTableEntry yaw;

	public static VisionManager getInstance() {
		return VisionManager.trackingInstance;
    }
    
    private VisionTarget lastTarget;
	private VisionManager() {
		//chameleon = NetworkTableInstance.getDefault().getTable("chameleon-vision");
		photon = NetworkTableInstance.getDefault().getTable("photonvision");
	}
	
	@Override
	public void update() {
		
	}
	
	public VisionTarget getTarget(){
		yaw = photon.getSubTable(VisionConstants.CAMERA_NAME).getEntry("targetYaw");
		synchronized (this) {
			//if(chameleon.getSubTable(VisionConstants.CAMERA_NAME).getEntry("isValid").getBoolean(false)){
			if(photon.getSubTable(VisionConstants.CAMERA_NAME).getEntry("hasTarget").getBoolean(false)){
				VisionTarget target = new VisionTarget((float)yaw.getDouble(0));
				lastTarget = target;
				return target;
			}else{
				return new VisionTarget(0);
			}
		}
	  }
    
    public VisionTarget getLastTarget() {
        return lastTarget;
    }
}