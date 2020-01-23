package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

/**
 * TeleDriveCommand
 */
public class TeleDriveCommand extends CommandBase {

  public static final double ROTATION_MULTIPLIER = .78;

  public static final double SLOW_MODE_SPEED_MULTIPLIER = .6;
  public static final double SLOW_MODE_ROTATION_MULTIPLIER = .9;

  private final Joystick driverL;
  private final Joystick driverR;
  private final Drive driveTrainSubsystem;

  public TeleDriveCommand(Joystick driverL, Joystick driverR, Drive driveTrainSubsystem) {
    this.driverL = driverL;
    this.driverR = driverR;
    this.driveTrainSubsystem = driveTrainSubsystem;
    addRequirements(driveTrainSubsystem);
  }

  @Override
  public void execute() {
    driveTrainSubsystem.tankDrive(driverL.getY(), driverR.getY());
    if (driverL.getTriggerPressed()) {
      savePose();
      System.out.println("Pose Saved");
    }
  }

  private void savePose() {
    driveTrainSubsystem.saveCurrentPose();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public void end(boolean interrupted) {
    driveTrainSubsystem.arcadeDrive(0, 0);
  }

}