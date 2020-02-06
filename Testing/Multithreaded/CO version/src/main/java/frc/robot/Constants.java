
package frc.robot;

import static frc.robot.Constants.DriveTrainConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.DriveTrainConstants.FEED_FORWARD;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class DriveTrainConstants {
    public static final int DEVICE_ID_RIGHT_MASTER = 0;
    public static final int DEVICE_ID_RIGHT_SLAVE = 1;
    public static final int DEVICE_ID_LEFT_MASTER = 2;
    public static final int DEVICE_ID_LEFT_SLAVE = 3;

    public static final int SENSOR_UNITS_PER_ROTATION = 4096;
    public static final double WHEEL_DIAMETER_INCHES = 6d;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;

    public static final double TRACK_WIDTH_METERS = 0.555625;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        TRACK_WIDTH_METERS);

    /** Voltage needed to overcome the motor’s static friction. kS */
    public static final double kS = 0.829;

    /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
    public static final double kV = 3.04;

    /** Voltage needed to induce a given acceleration in the motor shaft. kA */
    public static final double kA = 0.676;

    public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(kS, kV, kA);

    public static final double DEADBAND = 0.11;

    public static final double kP = 1.0;
    public static final double kD = 0.11;

    public static final double CLOSED_LOOP_RAMP = .2;
    public static final double OPEN_LOOP_RAMP = .25;
  }

  public static final class ControllerConstants {
    public static final int PORT_ID_DRIVER_CONTROLLER = 0;
    public static final int PORT_ID_OPERATOR_CONSOLE = 1;
  }

  public static final class TrajectoryConstants {

    // Max speed in meters per second
    public static final double MAX_SPEED_AUTO = 3;

    // Max acceleration in meters per second per second
    public static final double MAX_ACCELERATION_AUTO = 2;

    // Max voltage
    public static final double MAX_VOLTAGE_AUTO = 11;

    public static final DifferentialDriveVoltageConstraint VOLTAGE_CONSTRAINT = 
        new DifferentialDriveVoltageConstraint(FEED_FORWARD, DRIVE_KINEMATICS, MAX_VOLTAGE_AUTO);

    // Baseline values for a RAMSETE follower in units of meters and seconds
    public static final double RAMSETE_B = 2;
    public static final double RAMSETE_ZETA = 0.7;
  }

  public static final class TeleConstants {
    // Max speed to drive in teleop in meters per second
    public static final double MAX_SPEED_TELE = 3.25;

    // Max angular velocity in teleop in degrees per second
    public static final double MAX_ANGULAR_VEL = 320;
  }

  public static final class TurretConstants {
    public static final int DEVICE_ID_TURRET = 1;
    public static final double kP = 0.001;
    public static final double kI = 0.001;
    public static final double kD = 0.001;
    public static final double kF = 0.001;
    public static final int kTimeoutMs = 30;
    public static final int kSlotIdx = 0;
  	public static final int kPIDLoopIdx = 0;
  }

  public static final class VisionConstants {
    public static final String CAMERA_NAME = "USB Camera-B4.09.24.1";
  }

}