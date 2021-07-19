package frc.auton;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.subsystem.Drive;

public class SetDrivePath extends AutoCommand {

	private Trajectory robotPath;
	private boolean isReversed;
	private ArrayList<PathTrigger> triggers = new ArrayList<>();

	public SetDrivePath(Trajectory robotPath) {
		this(robotPath, true);
	}

	public SetDrivePath(Trajectory robotPath, boolean isBlocking) {
		System.out.println("Set Drive Path");
		this.robotPath = robotPath;
		this.setBlocking(isBlocking);
	}
	
	public SetDrivePath(Trajectory robotPath, boolean isBlocking, PathTrigger... triggers) {
		System.out.println("Set Drive Path");
		this.robotPath = robotPath;
		this.setBlocking(isBlocking);
		for (PathTrigger trigger : triggers) {
			this.triggers.add(trigger);
		}
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
		Drive.getInstance().setAutoPath(robotPath, triggers);
	}

}