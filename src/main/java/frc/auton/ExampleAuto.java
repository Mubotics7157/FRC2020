package frc.auton;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsystem.*;
import frc.utility.control.*;
import frc.utility.control.motion.Path;
import frc.utility.math.*;
import frc.utility.Threaded;

public class ExampleAuto extends TemplateAuto implements Runnable {
    double startX;
    public ExampleAuto(int side, double startX) { 
        //Start position
        super(new Translation2D(startX, side*46), side);
        this.startX = startX;
    }

    double turretFinishTime = 0;
    int stage = 0;

    @Override
    public void run() {
        Path p1 = new Path(here());
        p1.addPoint(new Translation2D(8*12+18, this.side*46), 60); //TEMPORARY
        p1.addPoint(new Translation2D(168+12*4, this.side*(3*12+18+0)), 120);
        p1.addPoint(new Translation2D(212+12*4+12-1.5+2 /*0*/ + 5 , this.side*(3*12+18-2/*0 */-1 -2)), 120);

        
        if(startX < 50) {
                Path dropOff = new Path(here());
                dropOff.addPoint(new Translation2D(8*12+18, this.side*46), 60);
                drive.setAutoPath(dropOff, false);
                while(!drive.isFinished()) if(isDead()) return;

                Path reverse = new Path(here());
                reverse.addPoint(new Translation2D(0, this.side*46), 60);
                drive.setAutoPath(reverse, true);

                Path rocket1 = new Path(here());
                rocket1.addPoint(new Translation2D(144+48*2+5, this.side*(94+2)), 160);
                rocket1.addPoint(new Translation2D(169+48*2+5-1 -5, this.side*(113+26+2-2)), 160);//-2
                drive.setAutoPath(rocket1, false);
                

            } 
        else drive.setAutoPath(p1, false);
        
        while(!drive.isFinished()) {
            if(isDead()) return;
            switch(stage) {
                case 0:

                    break;
                case 1:
                    if(Timer.getFPGATimestamp() - turretFinishTime > 0.5)
                    break;
            }
           // System.out.println(robotTracker.getOdometry().translationMat.getX() + " , " +robotTracker.getOdometry().translationMat.getY());
        };
        Path p2 = new Path(here());
        p2.addPoint(new Translation2D(13*8, this.side*135), 120-6);
        p2.addPoint(new Translation2D(25-4, this.side*136), 160-6); //X=27, Y=135
        drive.setAutoPath(p2, true);

        while(!drive.isFinished())if(isDead()) return;

        Path p3 = new Path(here());
        p3.addPoint(new Translation2D(168+12*3, this.side*(3*12+18+0)), 120);
        p3.addPoint(new Translation2D(212+12*4+12+24+4/*+4+8*/ , this.side*(3*12+18-2-1 - 3)), 120); //-0
        drive.setAutoPath(p3, false);
    }
}