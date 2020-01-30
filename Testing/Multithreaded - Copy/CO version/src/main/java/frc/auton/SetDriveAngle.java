package frc.auton;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import frc.subsystem.Drive;

public class SetDriveAngle extends AutoCommand {

	private Rotation2d angle;

	public SetDriveAngle(Rotation2d angle) {
		this.angle = angle;
		this.setBlocking(true);
	}

	@Override
	public void start() {
		Drive.getInstance().setRotation(angle);
		System.out.println("Angle: " + angle.getDegrees());
	}

	@Override
	public boolean isFinished() {
		return Drive.getInstance().isFinished();
	}

}