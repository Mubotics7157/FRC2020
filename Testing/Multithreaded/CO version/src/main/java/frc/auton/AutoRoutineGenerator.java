package frc.auton;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.subsystem.RobotTracker;
import frc.subsystem.Turret;

public class AutoRoutineGenerator {
	private static Translation2d robotStartPosition = new Translation2d(20, 115);
	private static Translation2d midFieldRightPosition = new Translation2d(240, -108);
	private static Translation2d midFieldLeftPosition = new Translation2d(240, 108);

	private static AutoRoutine initialDrive;

	public enum PathOption {
		SCALE, SWITCH, BOTH, FORWARD, NONE
	}

	public enum StartPosition {
		LEFT, CENTER, RIGHT
	}

	static {
		Translation2d START_LEFT = new Translation2d();
	}

	public static AutoRoutine getRoutine(int path) {
		AutoRoutine routine;
		switch(path) {
			case 0: routine = generateSimpleLine();
			case 1: routine = targetStart(); //unfilled
			case 2: routine = humanPlayerStart();
			default: routine = humanPlayerStart();
		}
		return routine;
	}

	public static AutoRoutine humanPlayerStart() {
		TrajectoryConfig config = createConfig(2,1, false);
		RobotTracker.getInstance().setOdometry(new Pose2d(0, 0, new Rotation2d(0)));
		Trajectory startToIntake2 = TrajectoryGenerator.generateTrajectory(
		  List.of(
			  new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)),
			  new Pose2d(2.58, 0.342, Rotation2d.fromDegrees(18))
		  ),
		  config);
		
		Trajectory shootToIntake3 = TrajectoryGenerator.generateTrajectory(
		  List.of(
			  new Pose2d(3.1732,-0.05, Rotation2d.fromDegrees(-63)),
			  new Pose2d(new Translation2d(3.23,1), Rotation2d.fromDegrees(0))
		  ),
		  config);

		initialDrive = new AutoRoutine();
		Pose2d startPos = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(
								new SetDrivePath(startToIntake2, true,
								PathTrigger.create(new SetTurretAngle(-37.414), 0.2),
								PathTrigger.create(new SetIntaking(true, true), 0.5))
								);
		initialDrive.addCommands(new Delay(2.3), new SetIntaking(false, true), new SetTurretOff(),
								new SetShooting(3390, 1319, 3, true), new SetIntaking(true, true), 
								new Delay(1), new SetIntaking(false, true), new SetShooting(3390, 1319, 3, true));
		//initialDrive.addCommands(
		//						new SetDrivePath(shootToIntake3, true,
		//										PathTrigger.create(new SetIntaking(true, true), 0.3)));
		return initialDrive;
	}

	public static AutoRoutine targetStart() {
		TrajectoryConfig config = createConfig(2,1, false);
		RobotTracker.getInstance().setOdometry(new Pose2d(0, 0, new Rotation2d(0)));
		Trajectory startToIntake2 = TrajectoryGenerator.generateTrajectory(
		  List.of(
			  new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)),
			  new Pose2d(2.58, 0.342, Rotation2d.fromDegrees(14.6))
		  ),
		  config);
		
		Trajectory shootToIntake3 = TrajectoryGenerator.generateTrajectory(
		  List.of(
			  new Pose2d(3.1732,-0.05, Rotation2d.fromDegrees(-63)),
			  new Pose2d(new Translation2d(3.23,1), Rotation2d.fromDegrees(0))
		  ),
		  config);

		initialDrive = new AutoRoutine();
		Pose2d startPos = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(
								new SetDrivePath(startToIntake2, true,
								PathTrigger.create(new SetTurretAngle(-34.014), 0.2),
								PathTrigger.create(new SetIntaking(true, true), 0.5))
								);
		initialDrive.addCommands(new Delay(2.3), new SetIntaking(false, false), new SetTurretOff(),
								new SetShooting(3375, 1294, 5, true));
		//initialDrive.addCommands(
		//						new SetDrivePath(shootToIntake3, true,
		//										PathTrigger.create(new SetIntaking(true, true), 0.3)));
		return initialDrive;
	}

	
	public static AutoRoutine generateSimpleLine() {
		TrajectoryConfig config = createConfig(2,2, false);
		RobotTracker.getInstance().setOdometry(new Pose2d(0, 0, new Rotation2d(0)));
		Trajectory startMove = TrajectoryGenerator.generateTrajectory(
		  List.of(
			  new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)),
			  new Pose2d(2,0, Rotation2d.fromDegrees(0))
		  ),
		  config);

		initialDrive = new AutoRoutine();
		Pose2d startPos = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(
								new SetDrivePath(startMove, true,
								PathTrigger.create(new SetTurretFieldLock(), 0),
								PathTrigger.create(new SetShooting(3000, 2000, 3, false), 0.8))
								);
		return initialDrive;
	}

	private static TrajectoryConfig createConfig(double v, double a, boolean reversed) {
		TrajectoryConfig config = new TrajectoryConfig(v, a);
		config.addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT);
		config.setKinematics(DriveTrainConstants.DRIVE_KINEMATICS);
		DifferentialDriveKinematicsConstraint kkk = new DifferentialDriveKinematicsConstraint
		(
			DriveTrainConstants.DRIVE_KINEMATICS, 
			TrajectoryConstants.MAX_SPEED_AUTO
		);
		config.addConstraint(kkk);
		config.setReversed(reversed);
		return config;
	}
}