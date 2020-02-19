
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
    public static final int DEVICE_ID_RIGHT_MASTER = 1;
    public static final int DEVICE_ID_RIGHT_SLAVE = 3;
    public static final int DEVICE_ID_LEFT_MASTER = 2;
    public static final int DEVICE_ID_LEFT_SLAVE = 0;
    public static final int DEVICE_ID_LEFT_SHIFTER = 0;
    public static final int DEVICE_ID_RIGHT_SHIFTER = 1;

    public static final int SENSOR_UNITS_PER_ROTATION = 2048;
    public static final double GEAR_DIVISOR = 9.6666666;
    public static final double WHEEL_DIAMETER_INCHES = 6d;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;

    public static final double TRACK_WIDTH_METERS = 2.01325014;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        TRACK_WIDTH_METERS);

    /** Voltage needed to overcome the motor’s static friction. kS */
    public static final double kS = 0.174;

    /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
    public static final double kV = 2.22;

    /** Voltage needed to induce a given acceleration in the motor shaft. kA */
    public static final double kA = 0.149;

    public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(kS, kV, kA);

    public static final double DEADBAND = 0;

    public static final double kP = 0.000252;
    public static final double kD = 0;

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

  public static final class FieldConstants {
    public static final double LENGTH_METERS = 15.98;
    public static final double WIDTH_METERS = 8.21;
    public static final double INTERPORT_METERS = 1; //placeholder
    public static final double HORZ_DIST_TO_PORT = 8; //placeholder
  }

  public static final class LidarConstants {
    public static final double CALIBRATION_OFFSET = -18;
    public static final int DIO_PORT = 0;
  }

  public static final class IndexerConstants {
    public static final int DEVICE_ID_INDEXER_CONVEYOR = 0;
    public static final int DEVICE_ID_INDEXER_SLAVE = 0;
    public static final int DEVICE_ID_CHUTE = 0;
    public static final int DEVICE_ID_INTAKE = 0;
    public static final int[] SOLENOID_IDS_INTAKE = {0,1};
  }

  public static final class ShooterConstants {
    public static int DEVICE_ID_SHOOTER_TOP = 0;
    public static int DEVICE_ID_SHOOTER_BOTTOM = 0;
    public static double kP = 0.00045;
    public static double kI = 0.00045;
    public static double kD = 0.00045;
    public static double kFF = 0.00045;
  }
}