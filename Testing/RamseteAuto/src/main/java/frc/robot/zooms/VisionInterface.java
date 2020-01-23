/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.zooms;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.utility.Threaded;
import frc.utility.VisionTarget;

/**
 * Add your docs here.
 */
public class VisionInterface extends Threaded {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("camera_name");

    /*
    targetX represents the horizontal angle
    targetY represents the vertical angle
    */
    NetworkTableEntry targetX;
    NetworkTableEntry targetY;
    NetworkTableEntry distance;
    @Override
    public void update() {
        targetX=table.getEntry("yaw");
        targetY=table.getEntry("pitch");
        distance = table.getEntry("distance");
    }

    public VisionTarget getTarget() {
        return new VisionTarget(targetX.getDouble(0), targetY.getDouble(0), distance.getDouble(0));
    }
}
