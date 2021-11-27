package frc.auton;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.subsystem.RobotTracker;
import frc.subsystem.Turret;
import frc.utility.Coordinate;
import frc.utility.FieldCooridnates;

public class AutoRoutineGenerator {
	private static Translation2d robotStartPosition = new Translation2d(20, 115);
	private static Translation2d midFieldRightPosition = new Translation2d(240, -108);
	private static Translation2d midFieldLeftPosition = new Translation2d(240, 108);

	private static AutoRoutine initialDrive;
	static FieldCooridnates fieldCooridnates = new FieldCooridnates();


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
		TrajectoryConfig config = createConfig(1.5,1, false);
		config.setStartVelocity(1.);
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
								 PathTrigger.create(new SetIntaking(false, true),.05),
						 		PathTrigger.create(new SetShooting(3500, 2000, 3, false), 0.8),
								PathTrigger.create(new SetIndexing(true),.85))
								// PathTrigger.create(new SetIntaking(true, true),.05))
		);
		return initialDrive;
	}

		public static AutoRoutine generateSimpleLineAutomated() {
		TrajectoryConfig config = createConfig(2,1.5, false);
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
								//PathTrigger.create(new SetTurretFieldLock(), 0),
								PathTrigger.create(new SetShooting(3, false),.8))
								);
		return initialDrive;
		}

	public static AutoRoutine SixBallAutoTrench(){ // 3 loaded + trench, can only start if in front of trench finish filling out
		TrajectoryConfig config = createConfig(2,1.5, true);
		config.setEndVelocity(0);
		Pose2d startPos = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(startPos);
		Trajectory moveToTrench = TrajectoryGenerator.generateTrajectory(
			List.of(
			new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0)),
			new Pose2d(new Translation2d(8.2,-.67), new Rotation2d(1.012,0))
			), config);

		initialDrive = new AutoRoutine();
		RobotTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands( 
			//new SetTurretFieldLock(),  
			new SetShooting(3, false)
			//PathTrigger.create(SetShooting(3000, 1000, 3, false)), //arbitrary num
			//new Delay(3));
		);
		initialDrive.addCommands(new SetDrivePath(moveToTrench,true,
		 PathTrigger.create(new SetIntaking(true,true), .3), 
		 PathTrigger.create(new SetIntaking(true,false),.65),
		 //PathTrigger.create(new SetIntaking(false, true, false), .65),
		 //PathTrigger.create(new SetShooting(3000, 1000, 3, false), .8))); //arbitrary num
		 PathTrigger.create(new SetShooting(3, false), .7))
		);

		 return initialDrive;

	}

	public static AutoRoutine simpleLine(){
		TrajectoryConfig config = createConfig(.5, .5, false);
		RobotTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory move = TrajectoryGenerator.generateTrajectory(
			List.of(
			new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)),
			new Pose2d(2,0,Rotation2d.fromDegrees(0))
		),
		config);

		initialDrive = new AutoRoutine();
		Pose2d startPos = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(new SetDrivePath(move,true/*, PathTrigger.create(new SetIndexing(true), .4))*/));
		return initialDrive;
	}
	public static AutoRoutine generateSimpleAngle() {
		TrajectoryConfig config = createConfig(1,1, false);
		config.setEndVelocity(0);
		RobotTracker.getInstance().setOdometry(new Pose2d(0, 0, new Rotation2d(0)));
		Trajectory startMove = TrajectoryGenerator.generateTrajectory(
		  List.of(
			  new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0)),
			  new Pose2d(1,0, Rotation2d.fromDegrees(0))
		  ),
		  config);

		initialDrive = new AutoRoutine();
		Pose2d startPos = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(startPos);
		initialDrive.addCommands(
								new SetDrivePath(startMove, true));
		return initialDrive;
	}
	public static AutoRoutine barrelRoutine(){
		TrajectoryConfig config = createConfig(2.5, 2,true );
		TrajectoryConfig endConfig = createConfig(2.5, 2, false);
		endConfig.setEndVelocity(0);
		config.setEndVelocity(2);
		Trajectory newWideTrajectory = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(5.333,-2.329, new Rotation2d(-2.329, -0.738)),
				new Pose2d(4.857,-4.123, new Rotation2d(-0.735, 0.027)),
				new Pose2d(2.995,-2.886, new Rotation2d(1.509,1.727)),
				new Pose2d(4.71,-2.129, new Rotation2d(1.412,0.344)),
				new Pose2d(6.885,-1.369, new Rotation2d(0.218, 1.796)),
				new Pose2d(5.421,-1.337, new Rotation2d(-0.353,-2.039)),			
				new Pose2d( 6.584,-3.512, new Rotation2d(1.003, -0.558)),
				new Pose2d(8.511,-3.286, new Rotation2d(-0.112, 1.144)),
				new Pose2d(7.201 ,-2.162 , new Rotation2d(-2.871, -0.032)),
				new Pose2d( 1.137,-1.919 , new Rotation2d(-0.532,0.02 ))
			), endConfig);
			initialDrive = new AutoRoutine();
		Pose2d start = new Pose2d(new Translation2d(0.878,-2.099), new Rotation2d(0, 0));
		RobotTracker.getInstance().setOdometry(start);
		initialDrive.addCommands(new SetDrivePath(newWideTrajectory,false));
		return initialDrive;
	}

	public static AutoRoutine redOne(){
		TrajectoryConfig config = createConfig(1, 1,false);
		config.setEndVelocity(1);
		TrajectoryConfig endconfig = createConfig(1, 1,false);
		Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
				List.of(
					new Pose2d(1.758,-1.432 , new Rotation2d(0.534,-0.204)),
					new Pose2d(2.342,-2.37 , new Rotation2d(0.245,-0.585)),
					new Pose2d(3.226,-3.525 , new Rotation2d(0.616,0.118)),
					new Pose2d(3.851,-2.873,new Rotation2d(0.625,1.835)), 
					new Pose2d(4.749,-0.969,new Rotation2d(0.816,0.027)),
					new Pose2d(8.773, -0.969, new Rotation2d(.258,0))
				), endconfig);

		initialDrive = new AutoRoutine();
		Pose2d start = new Pose2d(new Translation2d(0.849,-1.546), new Rotation2d(0,0));
		RobotTracker.getInstance().setOdometry(start);
			initialDrive.addCommands(new SetDrivePath(trajectory,true,
			PathTrigger.create(new SetIntaking(true,true), .01), 
			PathTrigger.create(new RunChute(true, true), .05)
			));
		return initialDrive;
	}

	public static AutoRoutine redTwo(){
		TrajectoryConfig config = createConfig(3.5, 2,false);
		config.setEndVelocity(0);
		RobotTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory leg1 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(new Translation2d(0,0),Rotation2d.fromDegrees(0)),
				new Pose2d(3.079775, -0.104601, Rotation2d.fromDegrees(-1)),
				new Pose2d(4.379519,1.940491 , Rotation2d.fromDegrees(51.600)),
				new Pose2d(7.968154,0.596197 , Rotation2d.fromDegrees(31.330000))
		),config);

		initialDrive = new AutoRoutine();
		Pose2d start = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(start);
		initialDrive.addCommands(new SetDrivePath(leg1,true,PathTrigger.create(
			new SetIntaking(true, true),.5)));
		return initialDrive;
	}

	public static AutoRoutine secondRed(){
		TrajectoryConfig config = createConfig(2,2,false);
		config.setEndVelocity(0);
		RobotTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory leg1 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(new Translation2d(0,0),Rotation2d.fromDegrees(0)),
				new Pose2d(0.915501, 0.078550, Rotation2d.fromDegrees(6.06)),
				new Pose2d(2.611811,-1.1203931 , Rotation2d.fromDegrees(-34.49)),
				new Pose2d(4.368612,-0.003472 , Rotation2d.fromDegrees(35)),
				new Pose2d(8.137649,0.580949 , Rotation2d.fromDegrees(13.534))
				
				
		),config);

		initialDrive = new AutoRoutine();
		Pose2d start = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(start);
		initialDrive.addCommands(new SetDrivePath(leg1,true,
			PathTrigger.create(new SetIntaking(true,true), .1), 
			PathTrigger.create(new RunChute(true, true), .05),
			PathTrigger.create(new SetIntaking(false,true), .7)
		));
		return initialDrive;
	}
	public static AutoRoutine slalomRoutine(){
		TrajectoryConfig config = createConfig(2, 2,true );
		TrajectoryConfig turnConfig = createConfig(1.5, 1.5, true);
		TrajectoryConfig endConfig = createConfig(1.5,1.5,true);
		TrajectoryConfig startConfig = createConfig(1.5,1.5,true);
		startConfig.setStartVelocity(0);
		startConfig.setEndVelocity(1.5);
		endConfig.setEndVelocity(0);
		config.setStartVelocity(1.5);
		turnConfig.setStartVelocity(1.5);
		config.setEndVelocity(1.5);
		turnConfig.setEndVelocity(1.5);
		RobotTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
			Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
				List.of(
					new Pose2d(2.154,-3.553 , new Rotation2d(0.2260,.364)),
					new Pose2d(3.116,-2.251 , new Rotation2d(0.822,0.588)),
					new Pose2d(6.473,-2.542, new Rotation2d(0.638,-1.329)),
					new Pose2d(7.888,-3.771, new Rotation2d(1.545,0.235)),
					new Pose2d(5.834,-3.957, new Rotation2d(-0.776,0.17)),
					new Pose2d(3.229,-3.609, new Rotation2d(-1.17,0.553)),
					new Pose2d(2.129,-2.873, new Rotation2d(-0.421,0.906)),
					new Pose2d(0.908,-2.113, new Rotation2d(-0.461,0.081))
				), endConfig);
		initialDrive = new AutoRoutine();
		Pose2d start = new Pose2d(new Translation2d(0.884,-3.812), new Rotation2d(0,0));
		RobotTracker.getInstance().setOdometry(start);
		initialDrive.addCommands(new SetDrivePath(trajectory,true));
		return initialDrive;
	}

	
public static AutoRoutine BouncePath() {
	TrajectoryConfig moveConfig = createConfig(1.7, .7, true);
	TrajectoryConfig backwards = createConfig(1.7, .7, false);
	moveConfig.setEndVelocity(1.5);
	backwards.setEndVelocity(1.5);
	TrajectoryConfig turnConfig = createConfig(2, .7, false);
	turnConfig.setEndVelocity(0);

	RobotTracker.getInstance().setOdometry(new Pose2d(0,0,Rotation2d.fromDegrees(0)));

	Trajectory leg1 = TrajectoryGenerator.generateTrajectory(
		List.of(

			new Pose2d(new Translation2d(0,0),Rotation2d.fromDegrees(0)),
			new Pose2d(-1.909751, -1.144584, Rotation2d.fromDegrees(89.023987))
			

		)
	,moveConfig);

	Trajectory leg2 = TrajectoryGenerator.generateTrajectory(
		List.of(
			new Pose2d(-2.157335,-0.477802,Rotation2d.fromDegrees(107.919998)),
			new Pose2d(-2.614958, 0.023152, Rotation2d.fromDegrees(138.389)),
			new Pose2d(-2.665922,0.922003,Rotation2d.fromDegrees(91.039001)),
			new Pose2d(-4.133699,0.827626, Rotation2d.fromDegrees(-91.746002)),
			new Pose2d(-4.384727,-1.199028 , Rotation2d.fromDegrees(-97.136002))
		)
	,backwards);
	
	Trajectory leg3 = TrajectoryGenerator.generateTrajectory(
		List.of(
			new Pose2d(-4.436752, 1.345510, Rotation2d.fromDegrees(-88.585999)),
			new Pose2d(-6.739544,0.994054,Rotation2d.fromDegrees(90.610001)),
			new Pose2d(-6.753090,-1.633875 , Rotation2d.fromDegrees(92.753998))
		)
	,moveConfig);

	
	Trajectory leg4 = TrajectoryGenerator.generateTrajectory(
		List.of(
			new Pose2d(-6.524428,-1.504273,Rotation2d.fromDegrees(92.229996)),
			new	Pose2d(-7.709881,-0.270928,Rotation2d.fromDegrees(179.860001))
		)
,turnConfig);
	
		initialDrive = new AutoRoutine();
		Pose2d start = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(start);
		initialDrive.addCommands(new SetDrivePath(leg1), new SetDrivePath(leg2), new SetDrivePath(leg3), new SetDrivePath(leg4));
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