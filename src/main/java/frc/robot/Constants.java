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

	// IO IDs
	public static final int TurretLimitId = 0;
	
	// Controller

	public static final double[] MinControllerInput = {0.1, 0.08};
	public static final double MaxControllerInput = 1;
	public static final double MinControllerOutput = 0;
	public static final double MaxControllerOutput = 1;
	public static final double MaxAcceleration = 1000;
	public static final double DrivetrainEncoderTicksPerRotation = 42;
	public static final double DrivetrainGearingDivisor = 10.71;
	
	
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
	public static final double driveConversionFactor = WheelDiameter * Math.PI /
	DrivetrainGearingDivisor / DrivetrainEncoderTicksPerRotation;

	public static final double MinTurningRadius = 40;
	public static final double MinPathSpeed = 20;
	public static final double MaxPathSpeed = 120; //120
	public static final double MinLookAheadDistance = 14;
	public static final double MaxLookAheadDistance = 30;
	
	// Subsystems
	public static final int TimeoutMs = 10;
	
	// Drive
	public static final double kDriveInchesPerSecPerRPM = 2 * Math.PI/60d * Constants.WheelDiameter / DrivetrainGearingDivisor;
	public static final double maxTurnError = 2;
	public static final double maxPIDStopSpeed = 8;
	public static final double DriveHighSpeed = 190;
	public static final double DriveLowSpeed = 95;
	
	public static final double kDriveRightAutoP = 0; //0.00065
	public static final double kDriveRightAutoD = 0.000;
	public static final double kDriveRightAutoF = 0; //0.055
	public static final double kDriveLeftAutoP = 0;
	public static final double kDriveLeftAutoD = 0.000; //0.0001
	public static final double kDriveLeftAutoF = 0;

	//SOOPER SPECIAL FEEDFORWARD CONSTANTS
	public static final double kLv = 0.000; //kv
	public static final double kLa = 0.000; //ka
	public static final double kLVi = 0.000; //Vintercept

	
	public static final double kRv = 0.000; //kv
	public static final double kRa = 0.000; //ka
	public static final double kRVi = 0.000; //Vintercept
	

	public static final double kDriveRightHighP = kDriveRightAutoP;
	public static final double kDriveRightHighD = kDriveRightAutoD;
	public static final double kDriveRightHighF = kDriveRightAutoF;

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