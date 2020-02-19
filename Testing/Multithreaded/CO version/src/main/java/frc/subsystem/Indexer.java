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
        shooter = Shooter.getInstance();
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
                break;
        }
    }

    public synchronized IndexerState getIndexerState() {
        return indexerState;
    }

    public void setHungry(boolean hungry) {
        intakeSolenoid.set(hungry ? Value.kForward : Value.kReverse);
        intakeMotor.set(-1);
        indexerState = IndexerState.INTAKING;
    }

    public void toggleHungry() {
        boolean hungry = intakeSolenoid.get() == Value.kForward;
        intakeSolenoid.set(hungry ? Value.kReverse : Value.kForward);
        if (hungry) indexerState = IndexerState.INTAKING;
        else return;
    }

    public void chew() {
        slamLeft.set(1);
        slamRight.set(1);
    }

    public void spit() {
        slamLeft.set(-1);
        slamRight.set(1);
    }

    public void swallow() {
        whooshMotor.set(-1);
    }

    public void dropSoap() {
        soapBar.set(-1);
    }

    public void holdSoap() {
        soapBar.set(1);
    }

    public void feast() {
        if (lemons > 5) {
            indexerState = IndexerState.FULL;
            return;
        }
        chew(); // run conveyor
        swallow(); // run chute
        holdSoap();
    }

    public void setShooting() {
        indexerState = IndexerState.SHOOTING;
    }

    public void shoot() {
        
        dropSoap();
    }

    public synchronized int getLemonCount() {
        return lemons;
    }
}
