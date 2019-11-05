package frc.auton;
import java.util.ArrayList;
import java.util.Set;

import frc.subsystem.RobotTracker;
import frc.utility.control.motion.Path;
import frc.utility.math.Rotation2D;
import frc.utility.math.Translation2D;

public class AutoRoutineGenerator {
	private static Translation2D robotLeftStartPosition = new Translation2D(20, 115);

	private static Translation2D midFieldRightPosition = new Translation2D(240, -108);
	private static Translation2D midFieldLeftPosition = new Translation2D(240, 108);

	private static double reverseSpeed = 70;

	private static AutoRoutine toMidFieldRightReverse;
	private static AutoRoutine toMidFieldLeftReverse;
	private static AutoRoutine initialDrive;

	private static Position switchPos;
	private static Position scalePos;

	public enum Position {
		LEFT, RIGHT
	}

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
		toMidFieldRightReverse.addCommands(new DriveToPoints(reverseSpeed, true, midFieldRightPosition));

		toMidFieldLeftReverse = new AutoRoutine();
		toMidFieldLeftReverse.addCommands(new DriveToPoints(reverseSpeed, true, midFieldLeftPosition));
	}

	public static AutoRoutine generate() {
		AutoRoutine overallRoutine = new AutoRoutine();
		Path initialPath;
		initialDrive = new AutoRoutine();
        System.out.println("start left");
        RobotTracker.getInstance().setInitialTranslation(robotLeftStartPosition);
        
        initialPath = new Path(robotLeftStartPosition);
        initialPath.addPoint(180, 115, 120); //80
        initialPath.addPoint(270, 90, 120); //60

        initialDrive.addCommands(new SetDrivePath(initialPath, false));
        initialDrive.addCommands(new SetDriveAngle(new Translation2D(231, 80)));
        Path secondPath = new Path(new Translation2D(270, 90));
        secondPath.addPoint(228, 80, 50);
        
        Path thirdPath = new Path(new Translation2D(228, 78));
        thirdPath.addPoint(271, 97, 110); //change speed to 60
        
        Path fourthPath = new Path(new Translation2D(271, 97));
        fourthPath.addPoint(228, 87, 100); //240 87
        fourthPath.addPoint(213, 58, 45); //CHANGE THIS SPEED TO 50, POS 225, 56 | 219 56
        
        
        Path fifthPath = new Path(new Translation2D(218, 56)); //225 62
        fifthPath.addPoint(271, 97, 100);  //CHANGE THIS SPEED TO 50
        
        //end of 3 cube
        
        overallRoutine.addRoutines(initialDrive);
        overallRoutine.addCommands(new SetDrivePath(secondPath, false));
        overallRoutine.addCommands(new SetDrivePath(thirdPath, true));
        overallRoutine.addCommands(new SetDriveAngle(Rotation2D.fromDegrees(-55)));
        

        overallRoutine.addCommands(new Delay(0.5));
        /* 3 cube expiermental */
        overallRoutine.addCommands(new SetDriveAngle(new Translation2D(220, 58))); //226, 66
        overallRoutine.addCommands(new SetDrivePath(fourthPath, false));
        overallRoutine.addCommands(new SetDrivePath(fifthPath, true));
        overallRoutine.addCommands(new SetDriveAngle(Rotation2D.fromDegrees(-55))); //-55
        overallRoutine.addCommands(new Delay(3.0));
        
		return overallRoutine;
	}

	
	public static AutoRoutine generate2() {
		AutoRoutine overallRoutine = new AutoRoutine();
		Path initialPath;
		initialDrive = new AutoRoutine();
        System.out.println("start left");
        RobotTracker.getInstance().setInitialTranslation(new Translation2D(0,0));
        
        initialPath = new Path(new Translation2D(0,0));
        initialPath.addPoint(0, 60, 100); //80


        initialDrive.addCommands(new SetDrivePath(initialPath, false));
        overallRoutine.addRoutines(initialDrive);
		return overallRoutine;
	}

	public static AutoRoutine generate3() {
		AutoRoutine overallRoutine = new AutoRoutine();
		Path initialPath;
		initialDrive = new AutoRoutine();
        System.out.println("start left");
        RobotTracker.getInstance().setInitialTranslation(robotLeftStartPosition);
        
        initialPath = new Path(robotLeftStartPosition);
        initialPath.addPoint(180, 115, 120); //80
        initialPath.addPoint(270, 90, 120); //60

        initialDrive.addCommands(new SetDrivePath(initialPath, false));
        Path secondPath = new Path(new Translation2D(270, 90));
        secondPath.addPoint(228, 80, 50);
        
        Path thirdPath = new Path(new Translation2D(228, 78));
        thirdPath.addPoint(271, 97, 110); //change speed to 60
        
        Path fourthPath = new Path(new Translation2D(271, 97));
        fourthPath.addPoint(228, 87, 100); //240 87
        fourthPath.addPoint(213, 58, 45); //CHANGE THIS SPEED TO 50, POS 225, 56 | 219 56
        
        
        Path fifthPath = new Path(new Translation2D(218, 56)); //225 62
        fifthPath.addPoint(271, 97, 100);  //CHANGE THIS SPEED TO 50
        
        //end of 3 cube
        
        overallRoutine.addRoutines(initialDrive);
        overallRoutine.addCommands(new SetDrivePath(secondPath, false));
        overallRoutine.addCommands(new SetDrivePath(thirdPath, true));

        overallRoutine.addCommands(new Delay(0.5));
        /* 3 cube expiermental */        overallRoutine.addCommands(new SetDrivePath(fourthPath, false));
        overallRoutine.addCommands(new SetDrivePath(fifthPath, true));
        overallRoutine.addCommands(new Delay(3.0));
        
		return overallRoutine;
	}
}