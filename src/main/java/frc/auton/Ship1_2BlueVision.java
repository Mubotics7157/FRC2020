package frc.auton;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.subsystem.Manipulator.ManipulatorIntakeState;
import frc.subsystem.Manipulator.ManipulatorState;
import frc.utility.control.*;
import frc.utility.control.motion.Path;
import frc.utility.math.*;
import frc.utility.Threaded;
import frc.subsystem.Turret.*;
import frc.subsystem.Arm.*;

public class Ship1_2BlueVision extends TemplateAuto implements Runnable {
    double startX;
    public Ship1_2BlueVision(int side, double startX) { 
        //Start position
        super(new Translation2D(startX, side*46), side);
        this.startX = startX;
    }

    public void moveToRocket(boolean raiseElevator, int dir) {

    }

    double turretFinishTime = 0;
    int stage = 0;

    @Override
    public void run() {
        
        //Drive forward, blocking
        
        manipulator.setManipulatorState(ManipulatorState.HATCH);
        manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
        
        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(8*12+18, this.side*46), 60); //TEMPORARY
        p1.addPoint(new Translation2D(168+12*4, this.side*(3*12+18+0)), 120);
        p1.addPoint(new Translation2D(212+12*4+12-1.5+2 /*0*/ + 5 , this.side*(3*12+18-2/*0 */-1 -2)), 120);

        
        if(startX < 50) {
                Path dropOff = new Path(here());
                dropOff.addPoint(new Translation2D(8*12+18, this.side*46), 60);
                drive.setAutoPath(dropOff, false);
                while(!drive.isFinished())if(isDead()) return;

                double reverseTime = Timer.getFPGATimestamp();
                Path reverse = new Path(here());
                reverse.addPoint(new Translation2D(0, this.side*46), 60);
                drive.setAutoPath(reverse, true);
                while(Timer.getFPGATimestamp() - reverseTime < 0.5)if(isDead()) return;

                Path rocket1 = new Path(here());
                rocket1.addPoint(new Translation2D(144+48*2+5, this.side*(94+2)), 160);
                rocket1.addPoint(new Translation2D(169+48*2+5-1 -5, this.side*(113+26+2-2)), 160);//-2
                drive.setAutoPath(rocket1, false);
                

            } 
        else drive.setAutoPath(p1, false);
        
        turret.setDesired(this.side*-100, true);
        elevator.setHeight(Constants.HatchElevLow);
        while(!drive.isFinished()) {
            if(isDead()) return;
            switch(stage) {
                case 0:
                    if(turret.isFinished()) {
                        turretFinishTime = Timer.getFPGATimestamp();
                        stage++;
                    }
                    break;
                case 1:
                    if(Timer.getFPGATimestamp() - turretFinishTime > 0.5) manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
                    break;
            }
            manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
           // System.out.println(robotTracker.getOdometry().translationMat.getX() + " , " +robotTracker.getOdometry().translationMat.getY());
        }; 
        manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
        turret.setState(TurretState.VISION);
        
        while(!turret.isFinished() || !turret.isInRange()) {
            if(isDead()) return;
        }
        //turret.setState(TurretState.SETPOINT); temp
        //turret.restoreSetpoint();
        collisionManager.score();
        while(collisionManager.isScoring())if(isDead()) return;
        turret.setState(TurretState.SETPOINT);
        manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
        Path p2 = new Path(here());
        p2.addPoint(new Translation2D(13*8, this.side*135), 120-6);
        p2.addPoint(new Translation2D(25-4, this.side*136), 160-6); //X=27, Y=135
        drive.setAutoPath(p2, true);

        turret.setState(TurretState.SETPOINT);
        turret.setDesired(180, true);
        elevator.setHeight(Constants.HatchElevLow);
        while(!drive.isFinished())if(isDead()) return;
        
        turret.setState(TurretState.VISION);
        
        while(!turret.isFinished() || !turret.isInRange())if(isDead()) return;
        
        manipulator.setManipulatorIntakeState(ManipulatorIntakeState.INTAKE);
        arm.setState(ArmState.EXTEND);
        elevator.setHeight(Constants.HatchHP);
        double intakeAttemptTime = Timer.getFPGATimestamp();
        while(Timer.getFPGATimestamp()-intakeAttemptTime < 0.75)if(isDead()) return;
        //arm.setState(ArmState.RETRACT);
        collisionManager.retrieveHatch();
        //elevator.setHeight(Constants.HatchElevLow);
        while(Timer.getFPGATimestamp()-intakeAttemptTime < 1.5)if(isDead()) return;

        Path p3 = new Path(here());
        p3.addPoint(new Translation2D(168+12*3, this.side*(3*12+18+0)), 120);
        p3.addPoint(new Translation2D(212+12*4+12+24+4/*+4+8*/ , this.side*(3*12+18-2-1 - 3)), 120); //-0
        drive.setAutoPath(p3, false);

        turret.setState(TurretState.SETPOINT);
        turret.setDesired(this.side*-100, true);
        while(!turret.isFinished())if(isDead()) return;
        //manipulator.setManipulatorIntakeState(ManipulatorIntakeState.HATCH_HOLD);
        //turret.setState(TurretState.SETPOINT);
        
        while(!drive.isFinished())if(isDead()) return;
        turret.setState(TurretState.VISION);


        while(!turret.isFinished() || !turret.isInRange()) {
            if(isDead()) return;
        }

        collisionManager.score();
        while(collisionManager.isScoring())if(isDead()) return;
        /*
        Path p4 = new Path(here());
        p4.addPoint(new Translation2D(212+12*4+12+24+4+8 + 21.5, this.side*(3*12+18-2-1)), 120);
        p4.addPoint(new Translation2D(212+12*4+12+24+4+8 + 21.5, this.side*(3*12+18-2-1+24)), 120);
        drive.setAutoPath(p4, false);
        while(!drive.isFinished());
        collisionManager.extendBallIntake();
        while(!collisionManager.isBallIntakeOut());
        */
        /*
        //Drive forward whilst moving elevator, non blocking
        Path p2 = new Path(here());
        p2.addPoint(new Translation2D(50, 0), 40);
        drive.setAutoPath(p2, false);
        while(!drive.isFinished()) {
            if(p2.getPercentage() > 0.5) elevator.setHeight(400); 
        }; 

        //Move elevator, blocking
        elevator.setHeight(200);
        while(!elevator.isFinished()); 
        */
    }

    

}