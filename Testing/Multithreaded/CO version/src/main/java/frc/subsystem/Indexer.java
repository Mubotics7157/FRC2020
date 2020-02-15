/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.subsystem;

import com.cuforge.libcu.Lasershark;
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
    CANSparkMax slamMotor, slamSlave;
    CANSparkMax whooshMotor;
    DoubleSolenoid intakeSolenoid;
    Lasershark lemonCounter;

    private int lemons = 0;
    public static Indexer getInstance() {
        return instance;
    }

    public enum IndexerState {
        INTAKING, // want me some balls
        FULL, // keep balls in me
        NOPE // virgin bot
    }

    private IndexerState indexerState = IndexerState.NOPE;

    public Indexer() {
        intakeMotor = new CANSparkMax(IndexerConstants.DEVICE_ID_INTAKE, MotorType.kBrushless);
        slamMotor = new CANSparkMax(IndexerConstants.DEVICE_ID_INDEXER_CONVEYOR, MotorType.kBrushless);
        slamSlave = new CANSparkMax(IndexerConstants.DEVICE_ID_INDEXER_SLAVE, MotorType.kBrushless);
        whooshMotor = new CANSparkMax(IndexerConstants.DEVICE_ID_CHUTE, MotorType.kBrushless);
        intakeSolenoid = new DoubleSolenoid(IndexerConstants.SOLENOID_IDS_INTAKE[0], IndexerConstants.SOLENOID_IDS_INTAKE[1]);
        lemonCounter = new Lasershark(1);
        slamSlave.follow(slamMotor);
    }

    public synchronized IndexerState getIndexerState() {
        return indexerState;
    }

    public void setHungry(boolean hungry) {
        intakeSolenoid.set(hungry ? Value.kForward : Value.kReverse);
        intakeMotor.set(-1);
    }

    public void toggleHungry() {
        intakeSolenoid.set(intakeSolenoid.get() == Value.kForward ? Value.kReverse : Value.kForward);
    }

    public void chew() {
        slamMotor.set(1);
    }

    public void spit() {
        slamMotor.set(-1);
    }

    public void swallow() {
        whooshMotor.set(-1);
    }

    public void salivate() {
        indexerState = IndexerState.INTAKING;
        setHungry(true); // deploy intake
    }

    public void feast() {
        if (lemons > 5) {
            indexerState = IndexerState.FULL;
            return;
        }
        if (lemonCounter.getDistanceInches() >= 5) //make sure that a ball isn't indexed before running the conveyor again
        chew(); // run conveyor
        swallow(); // run chute
    }

    public synchronized int getLemonCount() {
        return lemons;
    }

    public void countLemons() {
        
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
        }
    }
}
