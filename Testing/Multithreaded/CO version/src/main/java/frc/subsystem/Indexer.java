/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IndexerConstants;
import frc.utility.Threaded;
import frc.utility.shooting.ShotGenerator;
import frc.utility.shooting.ShotGenerator.BACKSPINRATIOS;
import frc.utility.shooting.ShotGenerator.ShooterSpeed;

/**
 * Add your docs here.
 */
public class Indexer extends Threaded{
    private static final Indexer instance = new Indexer();
    CANSparkMax intakeMotor;
    CANSparkMax slamLeft, slamRight; // as facing intake
    CANSparkMax whooshMotor; // chute
    CANSparkMax soapBar; // shooter conveyor
    DoubleSolenoid intakeSolenoid;
    Shooter shooter;
    ShotGenerator shotGen;
    BACKSPINRATIOS backSpin = BACKSPINRATIOS.NORMAL;

    private int lemons = 0;
    public static Indexer getInstance() {
        return instance;
    }

    public enum IndexerState {
        INTAKING, // want me some balls
        FULL, // keep balls in me
        SHOOTING, // POOPOO
        NOPE // virgin bot
    }

    private IndexerState indexerState = IndexerState.NOPE;

    public Indexer() {
        intakeMotor = new CANSparkMax(IndexerConstants.DEVICE_ID_INTAKE, MotorType.kBrushless);
        slamLeft = new CANSparkMax(IndexerConstants.DEVICE_ID_INDEXER_CONVEYOR, MotorType.kBrushless);
        slamRight = new CANSparkMax(IndexerConstants.DEVICE_ID_INDEXER_SLAVE, MotorType.kBrushless);
        whooshMotor = new CANSparkMax(IndexerConstants.DEVICE_ID_CHUTE, MotorType.kBrushless);
        intakeSolenoid = new DoubleSolenoid(IndexerConstants.SOLENOID_IDS_INTAKE[0], IndexerConstants.SOLENOID_IDS_INTAKE[1]);
        soapBar = new CANSparkMax(IndexerConstants.DEVICE_ID_TOP_BELT, MotorType.kBrushless);
        shooter = new Shooter();
        shotGen = new ShotGenerator();
        SmartDashboard.putString("Intake State", "NOPE");
        slamLeft.setIdleMode(IdleMode.kCoast);
        slamRight.setIdleMode(IdleMode.kCoast);
        soapBar.setIdleMode(IdleMode.kCoast);
        whooshMotor.setOpenLoopRampRate(0.25);
        slamLeft.setOpenLoopRampRate(0.25);
        slamRight.setOpenLoopRampRate(0.25);
    }
    
    @Override
    public void update() {
        IndexerState snapIndexerState;
        synchronized (this) {
        snapIndexerState = indexerState;
        }
        switch (snapIndexerState) {
            case INTAKING:
                feast();
                SmartDashboard.putString("Intake State", "Intaking");
                break;
            case NOPE:
                SmartDashboard.putString("Intake State", "NOPE");
                break;
            case FULL:
                SmartDashboard.putString("Intake State", "Full");
                break;
            case SHOOTING:
                shoot(backSpin);
                SmartDashboard.putString("Intake State", "Shooting");
                break;
        }
    }

    public synchronized IndexerState getIndexerState() {
        return indexerState;
    }

    public synchronized void setHungry(boolean hungry) {
        System.out.println("the kkk");
        intakeSolenoid.set(hungry ? Value.kForward : Value.kReverse);
        if(hungry){
            indexerState = IndexerState.INTAKING;
        }else{
            indexerState = IndexerState.NOPE;
        }
        
    }

    public synchronized void setSalivation(boolean ww3) {
        intakeSolenoid.set(ww3 ? Value.kReverse : Value.kForward);
        indexerState = IndexerState.NOPE;
    }

    public synchronized void toggleHungry() {
        boolean hungry = intakeSolenoid.get() == Value.kForward;
        intakeSolenoid.set(hungry ? Value.kReverse : Value.kForward);
        if (hungry) indexerState = IndexerState.INTAKING;
        else return;
    }

    public void testShoot() {
        shooter.setSpeed(0.9, 0.3);
    }

    private void chew() {
        slamLeft.set(-0.5);
        slamRight.set(0.5);
    }

    private void spit() {
        slamLeft.set(0);
        slamRight.set(0);
        whooshMotor.set(0);
    }

    private void swallow() {
        whooshMotor.set(-1);
    }

    private void dropSoap() {
        soapBar.set(-1);
    }

    private void holdSoap() {
        soapBar.set(0.3);
    }

    private void feast() {
        chew(); // run conveyor
        intakeMotor.set(-1);
    }

    private void digest() {
        swallow(); // run chute
        holdSoap();
    }

    public synchronized void setShooting(BACKSPINRATIOS backSpin) {
        indexerState = IndexerState.SHOOTING;
        this.backSpin = backSpin;
    }
    
    public void setShooting() {
        indexerState = IndexerState.SHOOTING;
    }

    public void shoot(BACKSPINRATIOS backSpin) {
        ShooterSpeed shot = shotGen.getShot(Turret.getInstance().getDistanceToWall(), backSpin);
        if (shooter.atSpeed(shot.bottomSpeed, shot.topSpeed)) {
            dropSoap();
            chew();
            swallow();
        }
    }

    
    public void shootArbitrary(double bottomSpeed, double topSpeed) {
        if (shooter.atSpeed(bottomSpeed, topSpeed) && bottomSpeed != 0) {
            dropSoap();
            chew();
            swallow();
        }
        else {
            soapBar.set(0);
            //spit();
        }
    }

    public synchronized int getLemonCount() {
        return lemons;
    }
}
