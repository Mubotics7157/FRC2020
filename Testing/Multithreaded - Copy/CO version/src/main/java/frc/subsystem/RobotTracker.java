// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsystem;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.utility.InterpolablePoseCircularQueue;
import frc.utility.Threaded;
import frc.utility.math.InterpolablePair;
public class RobotTracker extends Threaded {

	private static final RobotTracker trackingInstance = new RobotTracker();

	public static RobotTracker getInstance() {
		return RobotTracker.trackingInstance;
	}

	private Drive drive;
	private Pose2d currentPose;
	private final DifferentialDriveOdometry differentialDriveOdometry;

	private Pose2d lastPose;
	private InterpolablePoseCircularQueue vehicleHistory;

	private RobotTracker() {
		vehicleHistory = new InterpolablePoseCircularQueue(100);
		drive = Drive.getInstance();
		differentialDriveOdometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(drive.getHeading()));
	}

	synchronized public Rotation2d getGyroAngle(long time) {
		return vehicleHistory.getInterpolatedPose(time).getRotation();
	}

	synchronized public Pose2d getOdometry() {
		return currentPose;
	}

	synchronized public void resetOdometry() {
		drive.zeroSensors();
		lastPose = new Pose2d(0, 0, Rotation2d.fromDegrees(drive.getHeading()));
		differentialDriveOdometry.resetPosition(lastPose, Rotation2d.fromDegrees(drive.getHeading()));
	  }

	synchronized public void setOdometry(Translation2d loc) {
		drive.zeroSensors();
		lastPose = new Pose2d(loc, Rotation2d.fromDegrees(drive.getHeading()));
		differentialDriveOdometry.resetPosition(lastPose, Rotation2d.fromDegrees(drive.getHeading()));
	}

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
			differentialDriveOdometry.update(
				Rotation2d.fromDegrees(heading),
				leftDist,
				rightDist);
			currentPose = differentialDriveOdometry.getPoseMeters();
			SmartDashboard.putString("Pose", differentialDriveOdometry.getPoseMeters().toString());
			vehicleHistory.add(new InterpolablePair<>(System.nanoTime(), currentPose));
		}
	}
}