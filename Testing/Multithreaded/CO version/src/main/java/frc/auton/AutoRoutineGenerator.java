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
		//TODO
	}

	public static AutoRoutine generate() {
		TrajectoryConfig config = new TrajectoryConfig(2, 1);
		config.addConstraint(TrajectoryConstants.VOLTAGE_CONSTRAINT);
		config.setKinematics(DriveTrainConstants.DRIVE_KINEMATICS);
		DifferentialDriveKinematicsConstraint kkk = new DifferentialDriveKinematicsConstraint
		(
			DriveTrainConstants.DRIVE_KINEMATICS, 
			TrajectoryConstants.MAX_SPEED_AUTO
		);
		config.addConstraint(kkk);
		config.setReversed(false);
		
		RobotTracker.getInstance().setOdometry(new Pose2d(0, 0, new Rotation2d(0)));
		Trajectory startToIntake2 = TrajectoryGenerator.generateTrajectory(
		  List.of(
			  new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)),
			  new Pose2d(3.1732,-0.05, Rotation2d.fromDegrees(-63))
		  ),
		  config);
		
		Trajectory shootToIntake3 = TrajectoryGenerator.generateTrajectory(
		  List.of(
			  new Pose2d(3.1732,-0.05, Rotation2d.fromDegrees(-63)),
			  new Pose2d(new Translation2d(3.2,1), Rotation2d.fromDegrees(0))
		  ),
		  config);

		initialDrive = new AutoRoutine();
		Pose2d startPos = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(new SetDrivePath(startToIntake2, true,
								PathTrigger.create(new SetShooting(), 0.6),
								PathTrigger.create(new SetIntaking(true, true), 0.7)));
		initialDrive.addCommands(new SetIntaking(false, false),
								new SetDrivePath(shootToIntake3, true,
												PathTrigger.create(new SetIntaking(true, true), 0.3)));
		return initialDrive;
	}
}