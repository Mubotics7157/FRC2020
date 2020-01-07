package frc.auton;

import frc.subsystem.Drive;
import frc.subsystem.RobotTracker;
import frc.utility.math.Rotation2D;
import frc.utility.math.Translation2D;

public class SetDriveAngle extends AutoCommand {

	private Rotation2D angle;
	private Translation2D point;

	public SetDriveAngle(Rotation2D angle) {
		this.angle = angle;
		this.setBlocking(true);
	}

	public SetDriveAngle(Translation2D point) {
		this.point = point;
		this.setBlocking(true);
	}

	@Override
	public void start() {
		if (angle == null) {
			angle = RobotTracker.getInstance().getOdometry().translationMat.getAngle(point);
		}
		Drive.getInstance().setRotation(angle);
		System.out.println("Angle: " + angle.getDegrees());
	}

	@Override
	public boolean isFinished() {
		return Drive.getInstance().isFinished();
	}

}