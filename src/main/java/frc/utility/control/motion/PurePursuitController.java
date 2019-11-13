// Copyright 2019 FRC Team 3476 Code Orange

package frc.utility.control.motion;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.subsystem.Drive;
import frc.subsystem.Drive.AutoDriveSignal;
import frc.subsystem.Drive.DriveSignal;
import frc.utility.OrangeUtility;
import frc.utility.control.RateLimiter;
import frc.utility.control.SynchronousPid;
import frc.utility.control.motion.Path.DrivingData;
import frc.utility.math.RigidTransform2D;
import frc.utility.math.Translation2D;

public class PurePursuitController {
	/*
	 * 1. Translation delta compared to robot 2. Find angle to path relative to
	 * robot 3. Drive towards point
	 */

	private Path robotPath;
	private boolean isReversed;
	private RateLimiter speedProfiler;
	private RateLimiter leftProfiler;
	private RateLimiter rightProfiler;

	public PurePursuitController(Path robotPath, boolean isReversed) {
		this.robotPath = robotPath;
		this.isReversed = isReversed;
		speedProfiler = new RateLimiter(Constants.PathAccel, Constants.PathJerk); //100, 10000
        
        //two new profilers
		leftProfiler = new RateLimiter(Constants.PathAccel, Constants.PathJerk);
		rightProfiler = new RateLimiter(Constants.PathAccel, Constants.PathJerk);
		if (robotPath.isEmpty()) {
		}
	}

	/**
	 * Calculates the look ahead and the desired speed for each side of the
	 * robot.
	 *
	 * @param robotPose
	 *            Robot position and gyro angle.
	 * @return Speed for each side of the robot.
	 *
	 */
	@SuppressWarnings("unchecked")
	public synchronized AutoDriveSignal calculate(RigidTransform2D robotPose) {
		if (isReversed) {
			robotPose = new RigidTransform2D(robotPose.translationMat, robotPose.rotationMat.flip());
		}
		double lookAheadDist = OrangeUtility.coercedNormalize(speedProfiler.getLatestValue(), Constants.MinPathSpeed,
				Constants.MaxPathSpeed, Constants.MinLookAheadDistance, Constants.MaxLookAheadDistance);
		DrivingData data = robotPath.getLookAheadPoint(robotPose.translationMat, lookAheadDist);
		if (data.remainingDist == 0.0) { // If robot passes point, remaining
											// distance is 0
			return new AutoDriveSignal(new DriveSignal(0, 0), true);
		}
        
        double robotSpeed = data.maxSpeed;
        /*
        Limit wheel speeds after
		double robotSpeed = speedProfiler.update(data.maxSpeed, data.remainingDist);
		if (robotSpeed < 20) {
			robotSpeed = 20;
		}
        */
		Translation2D robotToLookAhead = getRobotToLookAheadPoint(robotPose, data.lookAheadPoint);
        
		double radius;
		radius = getRadius(robotToLookAhead);
		SmartDashboard.putNumber("robotLookAheadx", robotToLookAhead.getX());
		SmartDashboard.putNumber("robotLookAheady", robotToLookAhead.getY());
		double delta = (robotSpeed / radius);
		double deltaSpeed = Constants.TrackRadius * delta;
		SmartDashboard.putNumber("delta v", deltaSpeed);
		SmartDashboard.putNumber("radius", radius);
		SmartDashboard.putNumber("speed", robotSpeed);

		if (isReversed) {
			robotSpeed *= -1;
		}
		double maxSpeed = Math.abs(robotSpeed) + Math.abs(deltaSpeed);
		if (maxSpeed > Constants.MaxPathSpeed) {
			robotSpeed -= Math.copySign(maxSpeed - Constants.MaxPathSpeed, robotSpeed);
		}
        double leftSpeed = leftProfiler.update(robotSpeed - deltaSpeed);
        double rightSpeed = rightProfiler.update(robotSpeed + deltaSpeed);
		return new AutoDriveSignal(new DriveSignal(leftSpeed, leftProfiler.getAccel(), rightSpeed, rightProfiler.getAccel()), false);
	}

	private double getRadius(Translation2D robotToLookAheadPoint) {
		// Hypotenuse^2 / (2 * X)
		double radius = Math.pow(Math.hypot(robotToLookAheadPoint.getX(), robotToLookAheadPoint.getY()), 2)
				/ (2 * robotToLookAheadPoint.getY());
		return radius;
	}

	private Translation2D getRobotToLookAheadPoint(RigidTransform2D robotPose, Translation2D lookAheadPoint) {
		Translation2D lookAheadPointToRobot = robotPose.translationMat.inverse().translateBy(lookAheadPoint);
		lookAheadPointToRobot = lookAheadPointToRobot.rotateBy(robotPose.rotationMat.inverse());
		return lookAheadPointToRobot;
	}

	/**
	 * Resets the time for the speed profiler.
	 */
	public void resetTime() {
		// TODO: Big Bang
		speedProfiler.reset();
	}

}
