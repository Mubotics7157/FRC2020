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
import frc.robot.Constants;
import frc.robot.Constants.IndexerConstants;
import frc.utility.LazyCANSparkMax;
import frc.utility.Threaded;
import frc.utility.shooting.ShotGenerator;
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
    private boolean lastAtSpeed = false;

    private boolean automated = true;
    private boolean swallow = false;

    private double botArbitrary = 0;
    private double topArbitrary = 0;

    private int lemons = 0;
    public static Indexer getInstance() {
        return instance;
    }

    public enum IndexerState {
        INTAKING, // want me some balls
        FULL, // keep balls in me
        SHOOTING, // POOPOO
        NOPE, // virgin bot
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
                break;
            case FULL:
                SmartDashboard.putString("Intake State", "Full");
                break;
            case SHOOTING:
                if (automated)
                    shoot();
                else {
                    shootArbitrary(botArbitrary, topArbitrary);
                }
                SmartDashboard.putString("Intake State", "Shooting");
                break;
        }
    }

    /**
     * Turns all indexer motors off
     */
    public synchronized void setOff() {
        indexerState = IndexerState.NOPE;
        spit();
        shootArbitrary(0, 0);
    }

    /**
     * Sets the intake to fold out and start running inwards. Also runs the indexer conveyors.
     * @param hungry true = run intake
     */
    public synchronized void setHungry(boolean hungry) {
        intakeSolenoid.set(hungry ? Value.kForward : Value.kReverse);
        if(hungry){
            indexerState = IndexerState.INTAKING;
        }else{
            setOff();
        }
    }
    /**
     * Sets if the chute runs while intaking
     */
    public synchronized void setSwallowing(boolean swallowing) {
        swallow = swallowing;
    }

    /**
     * Sets the intake to retracted or detracted. Sets indexer state to nope if intake is retracted.
     * @param ww3 true = intake out
     */
    public synchronized void setSalivation(boolean ww3) {
        if (!ww3) setOff();
        intakeSolenoid.set(ww3 ? IndexerConstants.INTAKE_DEPLOYED : IndexerConstants.INTAKE_RETRACTED);
    }

    public synchronized void toggleHungry() {
        boolean hungry = intakeSolenoid.get() == IndexerConstants.INTAKE_DEPLOYED;
        setHungry(!hungry);
    }

    /**
     * Sets the conveyor motors (V-shaped) to funnel balls towards the center
     */
    private void chew() {
        slamLeft.set(-0.3);
        slamRight.set(0.3);
    }

    /**
     * Stops all motors except shooter
     */
    private void spit() {
        slamLeft.set(0);
        slamRight.set(0);
        whooshMotor.set(0);
        intakeMotor.set(0);
        soapBar.set(0);
    }

    /**
     * Runs the chute upwards
     */
    private void swallow() {
        whooshMotor.set(-.5);
    }

    /**
     * Runs the soap bar upwards
     */
    private void dropSoap() {
        soapBar.set(-.5);
    }
    /**
     * Runs the soap bar downwards
     */
    private void holdSoap() {
        soapBar.set(0.3);
    }

    /**
     * Runs when intaking
     */
    private void feast() {
        if (swallow) swallow();
        chew();
        intakeMotor.set(-1);
    }

    /**
     * Puts indexed balls into the chute. Don't run if at least 2 balls aren't indexed.
     */
    private void digest() {
        swallow(); // run chute
        holdSoap();
    }

    /**
     * Shoots using auto generated wheel speeds
     */
    public void setShooting() {
        automated = true;
        indexerState = IndexerState.SHOOTING;
    }

    /**
     * Shoots with arbitrary values
     * @param bot bot wheel speed
     * @param top top wheel speed
     */
    public void setShooting(double bot, double top) {
        botArbitrary = bot;
        topArbitrary = top;
        indexerState = IndexerState.SHOOTING;
    }

    public void shoot() {
        ShooterSpeed shot = shotGen.getShot(Turret.getInstance().getDistanceToWall());
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
            spit();
        }
        SmartDashboard.putNumber("lemons", lemons);
        if(lastAtSpeed && !lemonShot){
            lemons--;
        }
        if (lemons <= 0) {
            setOff();
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
            setOff();
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
