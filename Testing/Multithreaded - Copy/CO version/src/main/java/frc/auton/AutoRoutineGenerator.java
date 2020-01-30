package frc.auton;
import java.util.ArrayList;
import java.util.Set;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import frc.subsystem.RobotTracker;

public class AutoRoutineGenerator {
	private static Translation2d robotLeftStartPosition = new Translation2d(20, 115);

	private static Translation2d midFieldRightPosition = new Translation2d(240, -108);
	private static Translation2d midFieldLeftPosition = new Translation2d(240, 108);

	private static double reverseSpeed = 70;

	private static AutoRoutine toMidFieldRightReverse;
	private static AutoRoutine toMidFieldLeftReverse;
	private static AutoRoutine initialDrive;

	public enum PathOption {
		SCALE, SWITCH, BOTH, FORWARD, NONE
	}

	public enum StartPosition {
		LEFT, CENTER, RIGHT
	}

	static {
		toMidFieldRightReverse = new AutoRoutine(); // Drives to Mid Field Right
													// Position from Current
													// Location
	}

	public static AutoRoutine generate3() {
		AutoRoutine overallRoutine = new AutoRoutine();
		initialDrive = new AutoRoutine();
		System.out.println("start left");
		Translation2d startPos = new Translation2d(0, 0);
		double robotVel = 60;
		RobotTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(new DriveToPoints(robotVel, false, RobotTracker.getInstance().getOdometry().getRotation(), 
		new Translation2d(0,1),
		new Translation2d(0,2)));
        
        overallRoutine.addRoutines(initialDrive);
        
		return overallRoutine;
	}
}