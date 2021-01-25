package frc.utility;

import java.util.ArrayList;
import java.util.List;

import javax.lang.model.util.ElementScanner6;

import frc.robot.Constants;

public class Coordinate{
    public double x;
    public double y;

    public Coordinate(double x, double y){
        x*=.3048;
        y*=.3048;
        this.x = x;
        if(y==Constants.MiscConstants.ROBOT_Y_OFFSET_METERS)
            this.y=y;
        else    
            this.y = y - Constants.MiscConstants.ROBOT_Y_OFFSET_METERS; 
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }
    
    public double getRelX(double fromPt){
        double difference = x-fromPt;
        return difference;
    }

    public double getRelY(double fromPt){
        double difference = y-fromPt;
        return difference;
    }

    public double getAngle(double x1, double y1){
        double radians = Math.atan2(y-y1,x-x1);
        double degrees = radians * (180/Math.PI);
        if(degrees<0)
            degrees +=360;
        return degrees;
    }
}