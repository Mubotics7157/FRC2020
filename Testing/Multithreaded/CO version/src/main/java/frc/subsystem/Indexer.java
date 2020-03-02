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
import frc.utility.LazyCANSparkMax;
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
    LazyCANSparkMax slamLeft, slamRight; // as facing intake
    LazyCANSparkMax whooshMotor; // chute
    LazyCANSparkMax soapBar; // shooter conveyor
    DoubleSolenoid intakeSolenoid;
    Shooter shooter;
    ShotGenerator shotGen;
    BACKSPINRATIOS backSpin = BACKSPINRATIOS.NORMAL;
    private boolean lastAtSpeed = false;

    private double botArbitrary = -1;
    private double topArbitrary = -1;

    private int lemons = 0;
    public static Indexer getInstance() {
        return instance;
    }

    public enum IndexerState {
        INTAKING, // want me some balls
        FULL, // keep balls in me
        SHOOTING, // POOPOO
        NOPE, // virgin bot
        ARB_SHOOT
    }

    private IndexerState indexerState = IndexerState.NOPE;

    public Indexer() {
        intakeMotor = new CANSparkMax(IndexerConstants.DEVICE_ID_INTAKE, MotorType.kBrushless);
        slamLeft = new LazyCANSparkMax(IndexerConstants.DEVICE_ID_INDEXER_CONVEYOR, MotorType.kBrushless);
        slamRight = new LazyCANSparkMax(IndexerConstants.DEVICE_ID_INDEXER_SLAVE, MotorType.kBrushless);
        whooshMotor = new LazyCANSparkMax(IndexerConstants.DEVICE_ID_CHUTE, MotorType.kBrushless);
        intakeSolenoid = new DoubleSolenoid(IndexerConstants.SOLENOID_IDS_INTAKE[0], IndexerConstants.SOLENOID_IDS_INTAKE[1]);
        soapBar = new LazyCANSparkMax(IndexerConstants.DEVICE_ID_TOP_BELT, MotorType.kBrushless);
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
                spit();
                break;
            case FULL:
                SmartDashboard.putString("Intake State", "Full");
                break;
            case SHOOTING:
                if (botArbitrary <= 0)
                    shoot(backSpin);
                else {
                    shootArbitrary(botArbitrary, topArbitrary);
                }
                SmartDashboard.putString("Intake State", "Shooting");
                break;
            case ARB_SHOOT:
                break;
        }
    }

    public synchronized IndexerState getIndexerState() {
        return indexerState;
    }

    public synchronized void setHungry(boolean hungry) {
        intakeSolenoid.set(hungry ? Value.kForward : Value.kReverse);
        if(hungry){
            indexerState = IndexerState.INTAKING;
        }else{
            spit();
            indexerState = IndexerState.NOPE;
            shootArbitrary(0, 0);
        }
        
    }

    public synchronized void setSalivation(boolean ww3) {
        intakeSolenoid.set(ww3 ? Value.kReverse : Value.kForward);
        if (!ww3) indexerState = IndexerState.NOPE;
    }

    public synchronized void toggleHungry() {
        boolean hungry = intakeSolenoid.get() == Value.kForward;
        intakeSolenoid.set(hungry ? Value.kReverse : Value.kForward);
        if (hungry) indexerState = IndexerState.INTAKING;
        else return;
    }

    public void testShoot() {
        shooter.setSpeed(0.7, 0.3);
    }

    public void debugStop() {
        slamLeft.set(0);
        slamRight.set(0);
        whooshMotor.set(0);
        shooter.setSpeed(0, 0);
        soapBar.set(0);
    }

    private void chew() {
        slamLeft.set(-0.3);
        slamRight.set(0.3);
    }

    private void spit() {
        slamLeft.set(0);
        slamRight.set(0);
        whooshMotor.set(0);
        intakeMotor.set(0);
        soapBar.set(0);
        //shootArbitrary(0, 0);
    }

    private void swallow() {
        whooshMotor.set(-.5);
    }

    private void dropSoap() {
        soapBar.set(-.5);
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
        botArbitrary = -1;
        topArbitrary = -1;
        indexerState = IndexerState.SHOOTING;
        this.backSpin = backSpin;
    }
    
    public void setShooting() {
        setShooting(BACKSPINRATIOS.NORMAL);
    }

    public void setShooting(double bot, double top) {
        botArbitrary = bot;
        topArbitrary = top;
        indexerState = IndexerState.SHOOTING;
    }

    public void shoot(BACKSPINRATIOS backSpin) {
        ShooterSpeed shot = shotGen.getShot(Turret.getInstance().getDistanceToWall(), backSpin);
        boolean atSpeed = shooter.atSpeed(shot.bottomSpeed, shot.topSpeed);
        boolean lemonShot = shooter.lemonShot();
        if(shot.bottomSpeed == 0){
            shooter.setSpeed(0, 0);
        }else if (atSpeed) {
            dropSoap();
            chew();
            swallow();
        }
        else {
            soapBar.set(0);
            spit();
        }
        SmartDashboard.putNumber("lemons", lemons);
        if(lastAtSpeed && !lemonShot){
            lemons--;
        }
        if (lemons <= 0) {
            indexerState = IndexerState.NOPE;
        }
        lastAtSpeed = lemonShot;
    }

    public void runAll() {
        dropSoap();
        chew();
        swallow();
    }

    public void setLemons(int lemons){
        this.lemons = lemons;
    }

    
    public void shootArbitrary(double bottomSpeed, double topSpeed) {
        boolean atSpeed = shooter.atSpeed(bottomSpeed, topSpeed);
        boolean lemonShot = shooter.lemonShot();
        if(bottomSpeed == 0){
            shooter.setSpeed(0, 0);
        }else if (atSpeed) {
            dropSoap();
            chew();
            swallow();
        }
        else {
            soapBar.set(0);
            spit();
        }
        SmartDashboard.putNumber("lemons", lemons);
        if(lastAtSpeed && !lemonShot){
            lemons--;
        }
        if (lemons <= 0) {
            indexerState = IndexerState.NOPE;
        }
        lastAtSpeed = lemonShot;
    }

    public void tuneMode(double bottomSpeed, double topSpeed) {
        //dropSoap();
        //chew();
        //shootArbitrary(bottomSpeed, topSpeed);
        swallow();
    }

    public synchronized int getLemonCount() {
        return lemons;
    }
}
