// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;


import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPipelineResult;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import frc.utility.Threaded;
import frc.utility.VisionTarget;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
public class VisionManager extends Threaded {

	private static final VisionManager trackingInstance = new VisionManager();
	//NetworkTable chameleon;
	NetworkTable photon;
	NetworkTableEntry yaw;
	PhotonCamera camera = new PhotonCamera(VisionConstants.CAMERA_NAME);
	PhotonPipelineResult result;
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
	  public synchronized boolean hasTarget(){

			return photon.getSubTable(VisionConstants.CAMERA_NAME).getEntry("hasTarget").getBoolean(false);
	  }
/*
	  public Pose2d getVisionEstimate(){
		//  var result = camera.getLatestResult();
		result = camera.getLatestResult();
		  if(camera.hasTargets()){
			  Transform2d camToTargeTransform2d = result.getBestTarget().getCameraToTarget();
			  Pose2d estCamPose = Constants.VisionConstants.FAR_TARGET_POSE.transformBy(camToTargeTransform2d.inverse());
			  return estCamPose;
		  }

		  else
		  	return null;
	  }*/

	  /*
	  public synchronized double getVisionLatency(){
		//  var result = camera.getLatestResult();
		result = camera.getLatestResult();
		if(camera.hasTargets()){
			double latency = Timer.getFPGATimestamp()-result.getLatencyMillis();
			return latency;
		}

		else
			return 0f;
	  }*/
    
    public VisionTarget getLastTarget() {
        return lastTarget;
	}
	
}