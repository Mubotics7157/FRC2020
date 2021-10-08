package frc.subsystem;

import java.util.function.Consumer;

import com.spartronics4915.lib.hardware.sensors.T265Camera;
import com.spartronics4915.lib.hardware.sensors.T265Camera.CameraUpdate;

//import com.spartronics4915.lib.hardware.sensors.T265Camera;
//import com.spartronics4915.lib.hardware.sensors.T265Camera.CameraUpdate;

import edu.wpi.first.wpilibj.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants;
import frc.utility.InterpolablePoseCircularQueue;
import frc.utility.Threaded;
import frc.utility.math.InterpolablePair;
public class RobotTracker extends Threaded{

	private static final RobotTracker trackingInstance = new RobotTracker();

	public static RobotTracker getInstance() {
		return RobotTracker.trackingInstance;
	}

	private Drive drive;
	private Pose2d currentPose;
	Pose2d estimatedPose2d;
	public final DifferentialDriveOdometry differentialDriveOdometry;
	private Pose2d lastPose;
	private InterpolablePoseCircularQueue vehicleHistory;
	private final DifferentialDrivePoseEstimator poseEstimator;
//d	private T265Camera cT265Camera = new T265Camera( new Transform2d(new Pose2d(0,0,new Rotation2d(0)), new Pose2d(Units.inchesToMeters(-11.75),
                                                            //Units.inchesToMeters(-4.75),
															//new Rotation2d())), 0.5);
															

	private RobotTracker() {
		vehicleHistory = new InterpolablePoseCircularQueue(100);
		drive = Drive.getInstance();
		differentialDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(drive.getHeading()));
		poseEstimator = new DifferentialDrivePoseEstimator(
			Drive.getInstance().getDriveRotation2d(),
			new Pose2d(),
			VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5), 0.01, 0.01),
          	VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(1)),
          	VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30))); // tune gains
	}

	synchronized public Rotation2d getGyroAngle(long time) {
		return vehicleHistory.getInterpolatedPose(time).getRotation();
	}

	synchronized public Pose2d getOdometry() {
		return currentPose;
	}

	synchronized public void resetOdometry() {
		drive.zeroSensors();
		lastPose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
		differentialDriveOdometry.resetPosition(lastPose, Rotation2d.fromDegrees(drive.getHeading()));
		currentPose = differentialDriveOdometry.getPoseMeters();
	}

	synchronized public void setOdometry(Pose2d loc) {
		drive.zeroSensors();
		differentialDriveOdometry.resetPosition(loc, Rotation2d.fromDegrees(drive.getHeading()));
		currentPose = differentialDriveOdometry.getPoseMeters();

	}

	/*private void updateFusedOdometry(){
		poseEstimator.update(
			Drive.getInstance().getDriveRotation2d(),
			Drive.getInstance().getRates(),
			Drive.getInstance().getLeftEncoderDistance(),
			Drive.getInstance().getRightEncoderDistance()

		);

		if(VisionManager.getInstance().hasTarget()){
			poseEstimator.addVisionMeasurement(VisionManager.getInstance().getVisionEstimate().transformBy(Constants.VisionConstants.kCameraToRobot), VisionManager.getInstance().getVisionLatency());
		}

		//poseEstimator.addVisionMeasurement(cT265Camera.sendOdometry(new Twist2);, timestampSeconds);
	}*/
	/**
	 * Integrates the encoders and gyro to figure out robot position. A constant
	 * curvature is assumed
	 */	
	@Override
	public void update() {
		double heading = drive.getHeading();
		double leftDist = drive.getLeftEncoderDistance();
		double rightDist = drive.getRightEncoderDistance();

		synchronized (this) {
			differentialDriveOdometry.update(Rotation2d.fromDegrees(heading), leftDist, rightDist);
			currentPose = differentialDriveOdometry.getPoseMeters();
			SmartDashboard.putNumber("PoseX", differentialDriveOdometry.getPoseMeters().getTranslation().getX());
			SmartDashboard.putNumber("PoseY", differentialDriveOdometry.getPoseMeters().getTranslation().getY());
			SmartDashboard.putNumber("PoseR", differentialDriveOdometry.getPoseMeters().getRotation().getDegrees());
			vehicleHistory.add(new InterpolablePair<>(System.nanoTime(), currentPose));
			/*
			updateFusedOdometry();
			estimatedPose2d = poseEstimator.getEstimatedPosition(); //output smartdashboard components for this
			SmartDashboard.putNumber("est PoseX", estimatedPose2d.getX());
			SmartDashboard.putNumber("est PoseY", estimatedPose2d.getY());
			SmartDashboard.putNumber("est PoseZ", estimatedPose2d.getRotation().getDegrees());
			*/
		}
	}
}