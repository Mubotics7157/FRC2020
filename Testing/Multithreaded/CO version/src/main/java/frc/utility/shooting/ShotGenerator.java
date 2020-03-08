/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.utility.shooting;

import java.util.Arrays;

import frc.robot.Constants.ShooterConstants;

/**
 * Add your docs here.
 */
public class ShotGenerator {
    private enum BACKSPINRATIOS{
        NORMAL(ShooterConstants.RATIO_NORMAL),
        FLOATY(ShooterConstants.RATIO_FLOATY),
        SINKY(ShooterConstants.RATIO_SINKY);

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
    private Double[][] normal = {
        {494d,495d,496d}, // X (distance)
        {3450d,9999d,10000d} // Y (RPM)
    };
    
    private Double[][] floaty = {
        {786d, 968d, 982d, 1052d, 1090d, 1215d}, // X (distance)
        {3245d, 3525d, 3545d, 3555d, 3575d, 3625d} // Y (RPM)
    };
    
    private Double[][] sinky = {
        {0d,1d,3d}, // X (distance)
        {0d,1d,2d} // Y (RPM)
    };

    SplineInterpolator normalInterpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(normal[1]), Arrays.asList(normal[0]));
    SplineInterpolator floatyInterpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(floaty[0]), Arrays.asList(floaty[1]));
    SplineInterpolator sinkyInterpolator = SplineInterpolator.createMonotoneCubicSpline(Arrays.asList(sinky[1]), Arrays.asList(sinky[0]));

    public ShooterSpeed getShot(double distance) {
        SplineInterpolator interpolator;
        BACKSPINRATIOS backSpin;
        if (distance > ShooterConstants.BACKSPIN_BREAKPOINT_CM) {
            backSpin = BACKSPINRATIOS.FLOATY;
        }
        else backSpin = BACKSPINRATIOS.NORMAL;

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
