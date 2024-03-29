
package frc.robot;

import static frc.robot.Constants.DriveTrainConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.DriveTrainConstants.FEED_FORWARD;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
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
    public static final int DEVICE_ID_RIGHT_MASTER = 10;
    public static final int DEVICE_ID_RIGHT_SLAVE = 9;
    public static final int DEVICE_ID_LEFT_MASTER = 2;
    public static final int DEVICE_ID_LEFT_SLAVE = 5;
    public static final int DEVICE_ID_LEFT_SHIFTER = 8;
    public static final int DEVICE_ID_RIGHT_SHIFTER = 9;

    public static final int SENSOR_UNITS_PER_ROTATION = 2048;
    public static final double GEAR_DIVISOR = 9.6666666;
    public static final double WHEEL_DIAMETER_INCHES = 6d;
    public static final double WHEEL_CIRCUMFERENCE_INCHES = WHEEL_DIAMETER_INCHES * Math.PI;
    public static final double WHEEL_CIRCUMFERENCE_METERS = Units.inchesToMeters(WHEEL_DIAMETER_INCHES) * Math.PI;
    public static final double ENCODER_METER_CONSTANT = (0.1524 * Math.PI) / 199.0879514239766 / 100;
    public static final double TRACK_WIDTH_METERS = .70979231; //.774763926;
    public static final double TRACK_WIDTH_FEET = 2.32871493;
    public static final DifferentialDriveKinematics DRIVE_KINEMATICS = new DifferentialDriveKinematics(
        TRACK_WIDTH_METERS);

    /** Voltage needed to overcome the motor’s static friction. kS */
    public static final double kS = .293;//.93; //.793

    /** Voltage needed to hold (or "cruise") at a given constant velocity. kV */
    public static final double kV = 2.2; //2.16s

    /** Voltage needed to induce a given acceleration in the motor shaft. kA */
    public static final double kA = .272; //.301

    public static final SimpleMotorFeedforward FEED_FORWARD = new SimpleMotorFeedforward(kS, kV, kA);

    public static final double DEADBAND = 0.15;

    public static final double kP =  .03;//0.139; //.03
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
    public static final int DEVICE_ID_TURRET = 7;
    public static final double kP = 15; //60 //13 
    public static final double kI = 0.00; //0
    public static final double kD = 17; //300 //15
    public static final double kF = 0; //0
    public static final int kTimeoutMs = 30;
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopIdx = 0;
    public static final int FORWARD_LIMIT_DEGREES = 190;
    public static final int REVERSE_LIMIT_DEGREES = -190;
    public static final int[] ANGLE_ID_SOLENOID = {2, 3};
    public static final double FORWARD_LIMIT_NATIVE = 190 * 4096 / 360; 
    public static final double REVERSE_LIMIT_NATIVE = -190 * 4096 / 360;
  }

  public static final class VisionConstants {
    public static final String CAMERA_NAME = "PS3";
    public static final double FAR_TARGET_X = Units.feetToMeters(54);
    public static final double FAR_TARGET_Y = Units.feetToMeters(27.0f/2.0f) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0/2.0);
    public static final Pose2d FAR_TARGET_POSE = new Pose2d(new Translation2d(FAR_TARGET_X, FAR_TARGET_Y), new Rotation2d(0));
    public static final Transform2d kCameraToRobot =
            new Transform2d(
                    new Translation2d(-0.25, 0), // in meters
                    new Rotation2d());
  }

  public static final class FieldConstants {
    public static final double LENGTH_METERS = 15.98;
    public static final double WIDTH_METERS = 8.21;
    public static final double INTERPORT_METERS = 1; //placeholder
    public static final double HORZ_DIST_TO_PORT = 8; //placeholder 
    public static final double ODOMETRY_OFFSET_X = 0;
    public static final double ODOMETRY_OFFSET_Y = 0;

  }

  public static final class LidarConstants {
    public static final double CALIBRATION_OFFSET = -18;
    public static final int DIO_PORT = 0;
    public static final double ANGLE_OFFSET = 2;
  }

  public static final class IndexerConstants {
    public static final int DEVICE_ID_TOP_BELT = 40;
    public static final int DEVICE_ID_INDEXER_CONVEYOR = 41;
  //  public static final int DEVICE_ID_INDEXER_SLAVE = 42;
    public static final int DEVICE_ID_CHUTE = 61;
    public static final int DEVICE_ID_INTAKE = 51;
    public static final int[] SOLENOID_IDS_INTAKE = {0,1};
    public static final Value INTAKE_DEPLOYED = Value.kReverse;
    public static final Value INTAKE_RETRACTED = Value.kForward;
  public static final int DEVICE_ID_INDEXER_BELT = 0;
    public static final boolean BROKEN = false;
    public static final boolean NOT_BROKEN = true;
  }

  public static final class ShooterConstants {
    public static int DEVICE_ID_SHOOTER_TOP = 42;
    public static int DEVICE_ID_SHOOTER_BOTTOM = 20; //18
    public static int MAX_ALLOWABLE_ERROR_RPM = 50;
    public static int MAX_ALLOWABLE_ERROR_RPM_FART = 300;
    public static int LEMON_ERROR_COUNTER = 100;
    //public static double kP_TOP = 0.0006;
    //public static double kI_TOP = 0.00;
    //public static double kD_TOP = 0.512;
    //public static double kFF_TOP = 0.000184;

    public static double kP_TOP = 0.0009;
    public static double kI_TOP = 0.000;
    public static double kD_TOP = 0.512;
    public static double kFF_TOP = 0.0002103;
    //public static double kP_BOTTOM = 0.0007;
    //public static double kI_BOTTOM = 0.00;
    //public static double kD_BOTTOM = 0.512;
    //public static double kFF_BOTTOM = 0.00019;

    public static double kP_BOTTOM = 0.0009;
    public static double kI_BOTTOM = 0.00;
    public static double kD_BOTTOM = 0.512; 
    public static double kFF_BOTTOM = 0.0002074;
    public static final int[] SOLENOID_IDS_SHOOTER = {2,3};
    public static final Value SHOOTER_RETRACTED = Value.kForward;
    public static final Value SHOOTER_ANGLED = Value.kReverse;
    public static final int BACKSPIN_BREAKPOINT_CM = 0;

    public static final double RATIO_FLOATY = 2.57;
    public static final double RATIO_NORMAL = 2.0597;
    public static final double RATIO_SINKY = 0.5;

  }

  public static final class ClimbConstants{
    public static final int DEVICE_ID_CLIMB = 6; //update later
    public static final double kP = 0;
    public static final double kI = 0.00; 
    public static final double kD = 0;
    public static final double kF = 0.00; 
    public static final int kTimeoutMs = 20; 
    public static final int kSlotIdx = 0;
    public static final int kPIDLoopx = 0;
    public static final int GEARING = 20;
  }
  
  public static final class MiscConstants{
    public static int LED = 1; //update later
    public static double ROBOT_Y_OFFSET_METERS = 2.286;
  }
}
