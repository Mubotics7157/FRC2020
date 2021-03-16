package frc.auton;
import java.nio.file.Path;
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

	public static AutoRoutine barrelRoutine(){
		TrajectoryConfig config = createConfig(3, 2,true );
		TrajectoryConfig endConfig = createConfig(2.5, 2, true);
		endConfig.setEndVelocity(0);
		config.setEndVelocity(0);
		RobotTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory move = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(new Translation2d(0,0),Rotation2d.fromDegrees(0)),
				new Pose2d(-2.470619,-0.082973,Rotation2d.fromDegrees(-10.91000)),
				new Pose2d(-4.021514,0.250229,Rotation2d.fromDegrees(-1.479996)),
				new Pose2d(-4.092075,0.330317,Rotation2d.fromDegrees(-30.529)),
				new Pose2d(-4.364941,1.535670,Rotation2d.fromDegrees(-120.850)),
				new Pose2d(-3.837267,1.551642,Rotation2d.fromDegrees(-168.430008)),
				new Pose2d(-2.992,1.566,Rotation2d.fromDegrees(137.190)),
				new Pose2d(-3.216,0.352,Rotation2d.fromDegrees(20.549)),
				new Pose2d(-5.816,-.074,Rotation2d.fromDegrees(6.224)),
				new Pose2d(-6.777,-.999,Rotation2d.fromDegrees(70.380)),
				new Pose2d(-6.093,-1.864,Rotation2d.fromDegrees(165.800)),
				new Pose2d(-5.035,-1.101,Rotation2d.fromDegrees(-104.670)),
				new Pose2d(-6.088,.3945,Rotation2d.fromDegrees(-39.530)),
				new Pose2d(-7.441,1.440,Rotation2d.fromDegrees(-1.390)),
				new Pose2d(-7.570534,0.235500,Rotation2d.fromDegrees(122)),
				new Pose2d(-8.972,1.233,Rotation2d.fromDegrees(122.160)),
				new Pose2d(-8.015,.0834, Rotation2d.fromDegrees(146.000))

		),endConfig);
			Trajectory leg1 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
				new Pose2d(-4.427721, 0.736240, Rotation2d.fromDegrees(-88.44))

		),endConfig);
			Trajectory leg2 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(-4.427721, 0.736240, Rotation2d.fromDegrees(-88.44)),
				new Pose2d(-2.771362, 1.093276, Rotation2d.fromDegrees(86.33))


		),endConfig);

		Trajectory leg3 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(-2.771362, 1.093276, Rotation2d.fromDegrees(86.33)),
				new Pose2d(-6.857051, -0.674395, Rotation2d.fromDegrees(91.216980)),
				new Pose2d(-4.934478,-1.075311 , Rotation2d.fromDegrees(-95.583)),
				new Pose2d(-7.429410, 1.627405, Rotation2d.fromDegrees(6.04))
		),endConfig);

			Trajectory leg4 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(-7.429410, 1.627405, Rotation2d.fromDegrees(6.04)),
				new Pose2d(-8.419118,0.257621 , Rotation2d.fromDegrees(151.10)),
				new Pose2d(-1.181883,-0.032069 , Rotation2d.fromDegrees(175.156998))

		),endConfig);

		Trajectory moveAll = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
				new Pose2d(-2.470619,-0.082973,Rotation2d.fromDegrees(-10.91000)),
				new Pose2d(-4.373016, 0.415173, Rotation2d.fromDegrees(-42)),
				new Pose2d(-2.623561, 1.406949, Rotation2d.fromDegrees(138.900009)),
				new Pose2d(-2.745443, -0.084210, Rotation2d.fromDegrees(59.343994)),
				new Pose2d(-6.857051, -0.674395, Rotation2d.fromDegrees(91.216980)),
				new Pose2d(-4.934478,-1.075311 , Rotation2d.fromDegrees(-95.583)),
				new Pose2d(-7.429410, 1.627405, Rotation2d.fromDegrees(6.04)),
				new Pose2d(-8.145571, 0.062183, Rotation2d.fromDegrees(150.43)),
				new Pose2d(-8.419118,0.257621 , Rotation2d.fromDegrees(151.10)),
				new Pose2d(-0.536926,0.295040, Rotation2d.fromDegrees(175.156998))

		),endConfig);



		
		


		initialDrive = new AutoRoutine();
		Pose2d start = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(start);
		//initialDrive.addCommands(new SetDrivePath(leg1,true), new SetDrivePath(leg2,true), new SetDrivePath(leg3,true), new SetDrivePath(leg4,true));
		initialDrive.addCommands(new SetDrivePath(moveAll,true));
		return initialDrive;
	}

	

	public static AutoRoutine routineThree(){
		TrajectoryConfig moveConfig = createConfig(2, 2, true);
		TrajectoryConfig turnConfig = createConfig(1.5, 1.5, true);
		TrajectoryConfig endConfig = createConfig(2, 2, true);
		moveConfig.setEndVelocity(1.5);
		turnConfig.setEndVelocity(1.5);
		endConfig.setEndVelocity(0);

		Trajectory move = TrajectoryGenerator.generateTrajectory(
			List.of(
	new Pose2d(-1.374599,-0.115002, Rotation2d.fromDegrees(12.781002)),
	new Pose2d(-2.029526,-1.391703, Rotation2d.fromDegrees(90.350998)),
	new Pose2d(-2.729339,1.335382, Rotation2d.fromDegrees(152.701004)),
	new Pose2d(-3.876381,1.567433, Rotation2d.fromDegrees(-146.958984)),
	new Pose2d(-4.482058,-0.910463,Rotation2d.fromDegrees(-95.299004)),
	new Pose2d(-5.500382,2.074891, Rotation2d.fromDegrees(-4.749000)),
	new Pose2d(-6.951861,-0.909220, Rotation2d.fromDegrees(85.451004)),
	new Pose2d(-8.897672,0.045091, Rotation2d.fromDegrees(-175.418991))
			),endConfig);

		Trajectory move1 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(0,0,Rotation2d.fromDegrees(0)),
				new Pose2d(-1.368,.122,Rotation2d.fromDegrees(-3.790))
			),moveConfig);

		Trajectory turn1 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(-1.368,.122,Rotation2d.fromDegrees(-3.790)),
				new Pose2d(-1.969,-1.207,Rotation2d.fromDegrees(82.340))
			),turnConfig);

		Trajectory move2 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(-1.969,-1.207,Rotation2d.fromDegrees(82.340)),
				new Pose2d(-2.305,1.189,Rotation2d.fromDegrees(130.059))
			),moveConfig);

		Trajectory turn2 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(-2.305,1.189,Rotation2d.fromDegrees(130.059)),
				new Pose2d(-3.151,1.665,Rotation2d.fromDegrees(167.100)),
				new Pose2d(-4.261,1.275,Rotation2d.fromDegrees(-85.113))

			),turnConfig);

		Trajectory move3 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(-4.261,1.275,Rotation2d.fromDegrees(-85.113)),
				new Pose2d(-4.332,-.991,Rotation2d.fromDegrees(-93.200)),
				new Pose2d(-4.300,1.044,Rotation2d.fromDegrees(-80.360))
			),moveConfig);

		Trajectory turn3 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(-4.300,1.044,Rotation2d.fromDegrees(-80.360)),
				new Pose2d(-5.749,1.803,Rotation2d.fromDegrees(26.260)),
				new Pose2d(-6.999,.968,Rotation2d.fromDegrees(90.360))
			),turnConfig);


		Trajectory move4 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(-6.999,.968,Rotation2d.fromDegrees(90.360)),
				new Pose2d(-7.066,-1.431,Rotation2d.fromDegrees(90.800)),
				new Pose2d(-8.520,.251,Rotation2d.fromDegrees(-178.3200))

			),endConfig);

		initialDrive = new AutoRoutine();
		Pose2d start = new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(start);
		initialDrive.addCommands(new SetDrivePath(move));
		return initialDrive;
		

	}
	public static AutoRoutine redOne(){
		TrajectoryConfig config = createConfig(3.5, 2.5,false);
		config.setEndVelocity(2.25);
		TrajectoryConfig endconfig = createConfig(3.5, 2.25,false);
		endconfig.setEndVelocity(0);
		RobotTracker.getInstance().setOdometry(new Pose2d(0,0,new Rotation2d(0)));
		Trajectory leg1 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(new Translation2d(0,0),Rotation2d.fromDegrees(0)),
				new Pose2d(3.079775, -0.104601, Rotation2d.fromDegrees(-1)),
				new Pose2d(4.571761, 2.233312, Rotation2d.fromDegrees(22.40)),
				new Pose2d(7.577031,0.004608, Rotation2d.fromDegrees(-30.210001)),
				new Pose2d(7.892087, 0.583988, Rotation2d.fromDegrees(-21.950001))
				//new Pose2d(2.943269,-0.160445 , Rotation2d.fromDegrees(-90))
	),endconfig);

		Trajectory leg2 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(4.571761, 2.233312, Rotation2d.fromDegrees(22.40)),
				new Pose2d(7.577031,0.004608, Rotation2d.fromDegrees(-30.210001))
			),endconfig);
		initialDrive = new AutoRoutine();
		Pose2d start = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(start);
		//initialDrive.addCommands(new SetDrivePath(leg1,true,PathTrigger.create(new SetIntaking(true, true),.25),PathTrigger.create(new SetIntaking(false,true), .27),PathTrigger.create(new SetIndexing(true), .35),PathTrigger.create(new SetIndexing(false), .4), PathTrigger.create(new SetIntaking(true, true),.5), PathTrigger.create(new SetIntaking(false,true), .56), PathTrigger.create(new SetIntaking(true,true), .8), PathTrigger.create(new SetIntaking(false,true), .86)));
		initialDrive.addCommands(
			new SetDrivePath(leg1,false, 
			PathTrigger.create(new SetIntaking(true,true), .1), 
			//PathTrigger.create(new SetIntaking(false,true), .5),
			PathTrigger.create(new RunChute(true, true), .1)
			//PathTrigger.create(new SetIntaking(true,true), .6),
//			PathTrigger.create(new SetIntaking(false,true), .75)

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
				//new Pose2d(2.943269,-0.160445 , Rotation2d.fromDegrees(-90))
		),config);

		initialDrive = new AutoRoutine();
		Pose2d start = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(start);
		initialDrive.addCommands(new SetDrivePath(leg1,true,PathTrigger.create(
			new SetIntaking(true, true),.5)));
			//PathTrigger.create(new SetIntaking(true, false), .27),
			//PathTrigger.create(new SetIndexing(true), .35),
			//PathTrigger.create(new SetIndexing(true), .45))
		//initialDrive.addCommands(new SetDrivePath(leg1,true));
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
			//PathTrigger.create(new RunChute(false,false), .9) 
			//PathTrigger.create(new SetIntaking(false,true), .5),
			//PathTrigger.create(new SetIntaking(false,true), .5),
			//PathTrigger.create(new SetIntaking(true, false), .27),
			//PathTrigger.create(new SetIndexing(true), .35),
			//PathTrigger.create(new SetIndexing(true), .45))
		//initialDrive.addCommands(new SetDrivePath(leg1,true));
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
		Trajectory move = TrajectoryGenerator.generateTrajectory(
			List.of(
				
				new Pose2d(-1.190,-.374,Rotation2d.fromDegrees(56.760)),
				new Pose2d(-2.198,-1.491,Rotation2d.fromDegrees(17.540)),
				new Pose2d(-4.912,-1.447,Rotation2d.fromDegrees(-2.435)),
				new Pose2d(-5.717,-.747,Rotation2d.fromDegrees(-74.531))
				/*new Pose2d(-6.546,.148,Rotation2d.fromDegrees(-4.778)),
				new Pose2d(-7.623,-.803,Rotation2d.fromDegrees(88.887)),
				new Pose2d(-6.225,-1.464,Rotation2d.fromDegrees(-176.01)),
				new Pose2d(-5.316,-.164,Rotation2d.fromDegrees(-116.762)),
				new Pose2d(-1.126,-.119,Rotation2d.fromDegrees(171.071)),
				new Pose2d(-.300,-1.529,Rotation2d.fromDegrees(118.443))
				*/




				
		),config);
		
		Trajectory move1 = TrajectoryGenerator.generateTrajectory(
			List.of(
			new Pose2d(new Translation2d(0,0),Rotation2d.fromDegrees(0)),
				new Pose2d(-1.190,-.374,Rotation2d.fromDegrees(56.760))

			),startConfig);

		Trajectory turn1 = TrajectoryGenerator.generateTrajectory(

		List.of(
				new Pose2d(-1.190,-.374,Rotation2d.fromDegrees(56.760)),
				new Pose2d(-2.198,-1.491,Rotation2d.fromDegrees(17.540))

		),turnConfig
		);

		Trajectory move2 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(-2.198,-1.491,Rotation2d.fromDegrees(17.540)),
				new Pose2d(-4.912,-1.447,Rotation2d.fromDegrees(-2.435))
			),config);

		Trajectory turn2 = TrajectoryGenerator.generateTrajectory(
			List.of(
				new Pose2d(-4.912,-1.447,Rotation2d.fromDegrees(-2.435)),
				new Pose2d(-6.025798,-0.833632,Rotation2d.fromDegrees(-80.351))

			),turnConfig);

			Trajectory move3 = TrajectoryGenerator.generateTrajectory(
				List.of(
				new Pose2d(-6.025798,-0.833632,Rotation2d.fromDegrees(-80.351)),
				new Pose2d(-6.546,.148,Rotation2d.fromDegrees(-4.778))

				),config);

			Trajectory turn3 = TrajectoryGenerator.generateTrajectory(
				List.of(
				new Pose2d(-6.546,.148,Rotation2d.fromDegrees(-4.778)),
				new Pose2d(-7.623,-.803,Rotation2d.fromDegrees(88.887))
					
				),turnConfig);

			Trajectory move4 = TrajectoryGenerator.generateTrajectory(
				List.of(
				new Pose2d(-7.623,-.803,Rotation2d.fromDegrees(88.887)),
				new Pose2d(-6.225,-1.464,Rotation2d.fromDegrees(-176.01))
				),config);

			Trajectory turn4 = TrajectoryGenerator.generateTrajectory(
				List.of(
				new Pose2d(-6.225,-1.464,Rotation2d.fromDegrees(-176.01)),
				//new Pose2d(-5.316,-.164,Rotation2d.fromDegrees(-116.762))
				new Pose2d(-6.029934,-0.647818, Rotation2d.fromDegrees(-86.586006))
					
			),turnConfig);

			Trajectory move5 = TrajectoryGenerator.generateTrajectory(
				List.of(
				
				new Pose2d(-6.029934,-0.647818, Rotation2d.fromDegrees(-86.586006)),
				//new Pose2d(-5.316,-.164,Rotation2d.fromDegrees(-116.762)),
				new Pose2d(-2.716390,0.410242,Rotation2d.fromDegrees(174.910004))
					
			),config);

			Trajectory move6 = TrajectoryGenerator.generateTrajectory(
				List.of(
				new Pose2d(-2.716390,0.410242,Rotation2d.fromDegrees(174.910004)),
				new Pose2d(-1.522499,-0.653756,Rotation2d.fromDegrees(90.190002)),
				new Pose2d(-0.540623,-1.250486, Rotation2d.fromDegrees(176.631012))

			),endConfig);
		initialDrive = new AutoRoutine();
		Pose2d start = new Pose2d(new Translation2d(0,0), Rotation2d.fromDegrees(0));
		RobotTracker.getInstance().setOdometry(start);
		initialDrive.addCommands(new SetDrivePath(move1,true), new SetDrivePath(turn1,true), new SetDrivePath(move2,true), new SetDrivePath(turn2), new SetDrivePath(move3), new SetDrivePath(turn3), new SetDrivePath(move4), new SetDrivePath(turn4),new SetDrivePath(move5),new SetDrivePath(move6));

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
			new Pose2d(-2.614958, 0.023152, Rotation2d.fromDegrees(138.389)),//142 /123
			new Pose2d(-2.665922,0.922003,Rotation2d.fromDegrees(91.039001)),
			//new Pose2d(-2.924118,1.006595, Rotation2d.fromDegrees(91.059998)),//1.097//94
			new Pose2d(-4.133699,0.827626, Rotation2d.fromDegrees(-91.746002)),
			new Pose2d(-4.384727,-1.199028 , Rotation2d.fromDegrees(-97.136002))//changeto91
		)
	,backwards);
	
	Trajectory leg3 = TrajectoryGenerator.generateTrajectory(
		List.of(
			new Pose2d(-4.436752, 1.345510, Rotation2d.fromDegrees(-88.585999)),//91
			//new Pose2d(-6.537744,1.136592 , Rotation2d.fromDegrees(89.354004)),
			new Pose2d(-6.739544,0.994054,Rotation2d.fromDegrees(90.610001)),
			new Pose2d(-6.753090,-1.633875 , Rotation2d.fromDegrees(92.753998))

		)
	,moveConfig);

	
	Trajectory leg4 = TrajectoryGenerator.generateTrajectory(
		List.of(
			new Pose2d(-6.524428,-1.504273,Rotation2d.fromDegrees(92.229996)),
			//new Pose2d(-6.739544,0.994054,Rotation2d.fromDegrees(90.610001)),
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

	/*
			new Pose2d(-1.549645,-0.235317, Rotation2d.fromDegrees(27.840))
new , Rotation2d.fromDegrees(Pose2d(t)),
new Pose2d(-2.174881,-1.162771, Rotation2d.fromDegrees(73.840)),
new Pose2d(-2.291026,0.186251, Rotation2d.fromDegrees(114.4899)),
new , Rotation2d.fromDegrees(Pose2d(t)),
new Pose2d(-3.574524,1.698192, Rotation2d.fromDegrees(174.80)),
new , Rotation2d.fromDegrees(Pose2d(t)),
new Pose2d(-4.312480,0.513299, Rotation2d.fromDegrees(-94.090)),
new Pose2d(-4.477667,-1.057623, Rotation2d.fromDegrees(-92.2000)),
new Pose2d(-4.464360,0.842966, Rotation2d.fromDegrees(-91.4000)),
new , Rotation2d.fromDegrees(Pose2d(t)),
new Pose2d(-5.596196,1.903959, Rotation2d.fromDegrees(-5.070)),
new , Rotation2d.fromDegrees(Pose2d(t)),
new Pose2d(-6.866048,1.075666, Rotation2d.fromDegrees(74.840)),
new Pose2d(-6.974258,-1.120324, Rotation2d.fromDegrees(86.070)),
new , Rotation2d.fromDegrees(Pose2d(t)),
new Pose2d(-8.087368,0.501344, Rotation2d.fromDegrees(171.0700))

	*/
}