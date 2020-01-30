package frc.auton;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.subsystem.Drive;

public class SetDrivePath extends AutoCommand {

	private Trajectory robotPath;
	private boolean isReversed;

	public SetDrivePath(Trajectory robotPath) {
		this(robotPath, true);
	}

	public SetDrivePath(Trajectory robotPath, boolean isBlocking) {
		System.out.println("Set Drive Path");
		this.robotPath = robotPath;
		this.setBlocking(isBlocking);
	}

	@Override
	public boolean isFinished() {
		if(Drive.getInstance().isFinished()) {
			System.out.println("pathcomplete");
			return true;
		}
		return false;
	}

	@Override
	public void start() {
		Drive.getInstance().setAutoPath(robotPath);

	}

}