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
        NORMAL(0),
        FLOATY(1),
        SINKY(2);

        private final int value;

        BACKSPINRATIOS(final int newValue) {
            value = newValue;
        }
        public int getValue() { return value; }
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
        {0d,1d}
    };
    
    public Double[][] floaty = {
        {0d,1d}
    };
    
    public Double[][] sinky = {
        {0d,1d}
    };

    public ShooterSpeed getShot(double distance, BACKSPINRATIOS backSpin) {
        SplineInterpolator interpolator;
        switch (backSpin) {
            case NORMAL:
                interpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(normal[1]), Arrays.asList(normal[0]));
                break;
            case FLOATY:
                interpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(floaty[1]), Arrays.asList(floaty[0]));
                break;
            case SINKY:
                interpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(sinky[1]), Arrays.asList(sinky[0]));
                break;
            default:
                return null;
        }

        double rpm = interpolator.interpolate(distance);
        return new ShooterSpeed(rpm, rpm/backSpin.getValue());
    }
}
