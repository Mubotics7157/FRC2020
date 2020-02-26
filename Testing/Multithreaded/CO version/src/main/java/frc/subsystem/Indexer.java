/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystem;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
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
                break;
            case NOPE:
                break;
            case FULL:
                break;
            case SHOOTING:
                shoot(backSpin);
                break;
        }
    }

    public synchronized IndexerState getIndexerState() {
        return indexerState;
    }

    public synchronized void setHungry(boolean hungry) {
        intakeSolenoid.set(hungry ? Value.kForward : Value.kReverse);

        if(hungry){
            intakeMotor.set(-1);
            chew();
            swallow();
            dropSoap();
            testShoot();
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

    public synchronized void lick() {
        intakeSolenoid.set(Value.kForward);
    }

    public synchronized void slime() {
        intakeSolenoid.set(Value.kReverse);
    }

    private void testShoot() {
        shooter.setSpeed(0.90, 0.25);
    }

    private void chew() {
        slamLeft.set(-0.5);
        slamRight.set(0.5);
    }

    private void spit() {
        slamLeft.set(1);
        slamRight.set(-1);
    }

    private void swallow() {
        whooshMotor.set(-0.5);
    }

    private void dropSoap() {
        soapBar.set(-1);
    }

    private void holdSoap() {
        soapBar.set(1);
    }

    private void feast() {
        if (lemons > 5) {
            indexerState = IndexerState.FULL;
            return;
        }
        chew(); // run conveyor
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

    public synchronized int getLemonCount() {
        return lemons;
    }
}
