/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.utility.shooting;

import java.util.Arrays;

/**
 * Add your docs here.
 */
public class ShotGenerator {
    public enum BACKSPINRATIOS{
        NORMAL(2.0597),
        FLOATY(2.57),
        SINKY(0.5);

        private final double value;

        BACKSPINRATIOS(final double newValue) {
            value = newValue;
        }
        public double getValue() { return value; }
    }


    public class ShooterSpeed {

        public final double topSpeed;
        public final double bottomSpeed; 
        public ShooterSpeed(double topSpeed, double bottomSpeed) { 
          this.topSpeed = topSpeed; 
          this.bottomSpeed = bottomSpeed;
        } 
    }


    //log data as {top wheel RPM, distance it made it in}
    public Double[][] normal = {
        {494d,495d,496d}, // X (distance)
        {3450d,9999d,10000d} // Y (RPM)
    };
    
    public Double[][] floaty = {
        {968d,1090d,1215d}, // X (distance)
        {3500d,3550d,3600d} // Y (RPM)
    };
    
    public Double[][] sinky = {
        {0d,1d,3d}, // X (distance)
        {0d,1d,2d} // Y (RPM)
    };

    SplineInterpolator normalInterpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(normal[1]), Arrays.asList(normal[0]));
    SplineInterpolator floatyInterpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(floaty[0]), Arrays.asList(floaty[1]));
    SplineInterpolator sinkyInterpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(sinky[1]), Arrays.asList(sinky[0]));

    public ShooterSpeed getShot(double distance, BACKSPINRATIOS backSpin) {
        SplineInterpolator interpolator;
        switch (backSpin) {
            case NORMAL:
                interpolator = normalInterpolator;
                break;
            case FLOATY:
                interpolator = floatyInterpolator;
                break;
            case SINKY:
                interpolator = sinkyInterpolator;
                break;
            default:
                return null;
        }

        double rpm = interpolator.interpolate(distance);
        return new ShooterSpeed(rpm/backSpin.getValue(), rpm);
    }
}
