package frc.utility;

public class VisionTarget {

    public double x;
    public double y;
    public double distance;
    public double turretRelativeDistance;
    public double loc_x;
    public double loc_y;

    public VisionTarget(double x, double y, double turretRelativeDistance) {
        this.x = x;
        this.y = y;
        this.turretRelativeDistance = turretRelativeDistance;
        
    }

    public VisionTarget(VisionTarget v) {
        this.x = v.x;
        this.y = v.y;
        this.distance = v.distance;
        this.turretRelativeDistance = v.turretRelativeDistance;
        this.loc_x = v.loc_x;
        this.loc_y = v.loc_y;
    }

    public double getX() {
        return this.x;
    }
    
    public double getY() {
        return this.y;
    }

    public double getTurretDistance() {
        return this.turretRelativeDistance;
    }

    public void setTurretRelativeDistance(double d) {
        turretRelativeDistance = d;
    }

    public void setLoc(double x, double y) {
        this.loc_x = x;
        this.loc_y = y;
    }

}