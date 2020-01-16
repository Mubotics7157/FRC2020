package frc.auton;

import java.util.ArrayList;

import frc.subsystem.Drive;
import frc.subsystem.RobotTracker;
import frc.utility.control.motion.Path;
import frc.utility.math.Translation2D;

public class DriveToPoints extends AutoCommand {

	private ArrayList<Translation2D> points;
	private double speed;
	private boolean isReversed;

	public DriveToPoints(double speed, boolean isReversed, Translation2D... points) {
		this.points = new ArrayList<Translation2D>();
		this.speed = speed;
		this.isReversed = isReversed;
		setBlocking(true);
		for (Translation2D point : points) {
			this.points.add(point);
		}
	}

	@Override
	public void start() {
		System.out.println("Drive To Points");
		Path drivePath = new Path(RobotTracker.getInstance().getOdometry().translationMat);
		for (Translation2D point : points) {
			drivePath.addPoint(point.getX(), point.getY(), speed);
		}
		Drive.getInstance().setAutoPath(drivePath, isReversed);
	}

	@Override
	public boolean isFinished() {
		return Drive.getInstance().isFinished();
	}

}