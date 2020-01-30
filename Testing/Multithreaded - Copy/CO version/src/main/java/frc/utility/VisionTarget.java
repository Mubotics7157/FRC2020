package frc.utility;

public class VisionTarget {

    public float yaw;
    public double turretRelativeDistance;
    public double loc_x;
    public double loc_y;

    public VisionTarget(float yaw, float turretRelativeDistance) {
        this.yaw = yaw;
        this.turretRelativeDistance = turretRelativeDistance;
    }
    
    public VisionTarget(float yaw, float loc_x, float loc_y) {
        this.yaw = yaw;
        this.loc_x = loc_x;
        this.loc_y = loc_y;
    }

    public VisionTarget(VisionTarget v) {
        this.yaw = v.yaw;
        this.turretRelativeDistance = v.turretRelativeDistance;
        this.loc_x = v.loc_x;
        this.loc_y = v.loc_y;
    }

    public float getYaw() {
        return this.yaw;
    }

    public double getTurretDistance() {
        return Math.hypot(this.loc_x, this.loc_y);
    }

}