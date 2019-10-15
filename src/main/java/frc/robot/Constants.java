// Copyright 2019 FRC Team 3476 Code Orange

package frc.robot;

public final class Constants {

	public static final boolean steeringWheel = true;
	// Networking
	public static final int TelemetryPort = 5810;
	public static final String DriverStationIPv4 = "10.71.57.5"; // Temporary
	public static final int JetsonPort = 5805;
	public static final String JetsonIPv4 = "10.34.76.8";

	// CAN IDs
	public static final int DriveLeftMasterId = 3;
	public static final int DriveLeftSlave1Id = 4;
	public static final int DriveLeftSlave2Id = 14;//not currently used
	public static final int DriveRightMasterId = 5;
	public static final int DriveRightSlave1Id = 6;
	public static final int DriveRightSlave2Id = 13;//not currently used

	

	public static final int HatchIntakeMotorId = 23;
	public static final int HatchIntakeDeployMotorId = 22;

	public static final int ElevatorMasterId = 9;
	public static final int ElevatorSlaveId = 8;

	public static final int ManipulatorMotor1Id = 31;
	public static final int ManipulatorMotor2Id = 32;
	
	public static final int ClimberMasterId = 15;
	public static final int ClimberSlaveId = 16;
	
	// PCM IDs
	public static final int DriveShifterSolenoidId = 4;
	public static final int BallIntakeSolenoidId = 7;
	public static final int ArmSolenoidId = 5;
	public static final int ManipulatorSolenoidId = 6;

	// IO IDs
	public static final int TurretLimitId = 0;
	
	// Controller

	public static final double[] MinControllerInput = {0.15, 0.08};
	public static final double MaxControllerInput = 1;
	public static final double MinControllerOutput = 0;
	public static final double MaxControllerOutput = 1;
	public static final double MaxAcceleration = 1000;
	
	// General
	public static final double EncoderTicksPerRotation = 4096;
	public static final double DegreesPerEncoderTick = 360 * (1d / EncoderTicksPerRotation);
	public static final double EncoderTicksPerDegree = (1d / 360) * EncoderTicksPerRotation;

	public static final double ExpectedCurrentTolerance = 0;
	public static final double ExpectedRPMTolerance = 0;
	public static final double ExpectedPositionTolerance = 0;

	// Game
	public static final double RocketBaseHeight = 27.5;
	public static final double RocketMiddleHeight = 55.5;
	public static final double RocketTopHeight = 83.5;

	public static final double HatchPanelHeight = 2 + (1 / 6); // The height of each hatch panel
	
	// Autonomous Driving
	public static final double TrackRadius = -12;
	public static final double WheelDiameter = 6.0; //6.09; //expiermental
	public static final double MinTurningRadius = 40;
	public static final double MinPathSpeed = 20;
	public static final double MaxPathSpeed = 120; //120
	public static final double MinLookAheadDistance = 14;
	public static final double MaxLookAheadDistance = 30;
	
	// Subsystems
	public static final int TimeoutMs = 10;
	
	// Drive
	public static final double kDriveInchesPerSecPerRPM = 2 * Math.PI/60d * Constants.WheelDiameter/2d
	* 22d / 62d / 3d;
	public static final double maxTurnError = 2;
	public static final double maxPIDStopSpeed = 8;
	public static final double DriveHighSpeed = 190;
	public static final double DriveLowSpeed = 95;
	
	public static final double kDriveRightAutoP = 4*0.0005263 * kDriveInchesPerSecPerRPM; //0.00065
	public static final double kDriveRightAutoD = 0.000; 
	public static final double kDriveRightAutoF = 1/193.12283370478679  * kDriveInchesPerSecPerRPM; //0.055
	public static final double kDriveLeftAutoP = 4*0.0005263 * kDriveInchesPerSecPerRPM;
	public static final double kDriveLeftAutoD = 0.000; //0.0001
	public static final double kDriveLeftAutoF = 1/203.7763632654868 * kDriveInchesPerSecPerRPM ; //0.0005 too high

	public static final double kDriveRightHighP = kDriveRightAutoP;
	public static final double kDriveRightHighD = kDriveRightAutoD;
	public static final double kDriveRightHighF = kDriveRightAutoF;
	public static final double kDriveRightHighFIntercept = 0;
	public static final double kDriveRightHighA = 0;
	public static final double kDriveRightLowP = 0;
	public static final double kDriveRightLowD = 0;
	public static final double kDriveRightLowF = 0;
	public static final double kDriveRightLowFIntercept = 0;
	public static final double kDriveRightLowA = 0;
	
	public static final double kDriveLeftHighP = kDriveLeftAutoP;
	public static final double kDriveLeftHighD = kDriveLeftAutoD;
	public static final double kDriveLeftHighF = kDriveLeftAutoF;
	public static final double kDriveLeftHighFIntercept = 0;
	public static final double kDriveLeftHighA = 0;
	public static final double kDriveLeftLowP = 0;
	public static final double kDriveLeftLowD = 0;
	public static final double kDriveLeftLowF = 0;
	public static final double kDriveLeftLowFIntercept = 0;
	public static final double kDriveLeftLowA = 0;
	public static final double kHoldP = 4;
	
	

	public static final double DriveTeleopAccLimit = 120;
	public static final double DriveTeleopJerkLimit = 2000;
	public static final double DriveExpectedCurrent = 1.5;
	public static final double DriveExpectedRPM = 0;
	public static final double DriveExpectedPosition = 0;
	//COMP
	public static final double cameraYOffset = 6.36;//5.310 + 1.25;//5.310 + 1.25;
	public static final double cameraXOffset = -4.75;//-4.815 + 1.6 - 1.0;

	private Constants() {
	}
}