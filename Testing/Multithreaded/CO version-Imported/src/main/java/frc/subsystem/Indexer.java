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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.TurretConstants;
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
    LazyCANSparkMax slamMotor; // as facing intake
    LazyCANSparkMax slamSlave;
    LazyCANSparkMax whooshMotor; // chute
    LazyCANSparkMax soapBar; // shooter conveyor
    DoubleSolenoid intakeSolenoid;
    DoubleSolenoid shooterSolenoid;
    Shooter shooter;
    ShotGenerator shotGen;
    private boolean lastAtSpeed = false;

    private boolean automated = true;
    private boolean swallow = false;
    private boolean intake = true;

    private double botArbitrary = 0;
    private double topArbitrary = 0;

    private double topRPMAdjust = 0;
    private double botRPMAdjust = 0;
    private double rpmRatio = 2.57;

    private boolean turretUp = false;
    private double intakeSpeed = -1;
    private boolean innerPort = false;

    private boolean passed;

    private int lemons = 0;

    private DigitalInput breakBeam;

    public static Indexer getInstance() {
        return instance;
    }

    public enum IndexerState {
        INTAKING, // want me some balls
        PUKE, // keep balls in me
        SHOOTING, // POOPOO
        NOPE, // virgin bot
        REVVING,
        INDEXING
    }

    private IndexerState indexerState = IndexerState.NOPE;

    public Indexer() {
        intakeMotor = new CANSparkMax(IndexerConstants.DEVICE_ID_INTAKE, MotorType.kBrushless);
        slamMotor = new LazyCANSparkMax(IndexerConstants.DEVICE_ID_INDEXER_CONVEYOR, MotorType.kBrushless);
        //slamSlave = new LazyCANSparkMax(IndexerConstants.DEVICE_ID_INDEXER_SLAVE, MotorType.kBrushless);
        whooshMotor = new LazyCANSparkMax(IndexerConstants.DEVICE_ID_CHUTE, MotorType.kBrushless);
        intakeSolenoid = new DoubleSolenoid(IndexerConstants.SOLENOID_IDS_INTAKE[0], IndexerConstants.SOLENOID_IDS_INTAKE[1]);
        soapBar = new LazyCANSparkMax(IndexerConstants.DEVICE_ID_TOP_BELT, MotorType.kBrushless);
        shooterSolenoid = new DoubleSolenoid(TurretConstants.ANGLE_ID_SOLENOID[0], TurretConstants.ANGLE_ID_SOLENOID[1]);

        breakBeam = new DigitalInput(1);


        shooter = new Shooter();
        shotGen = new ShotGenerator();
        
        SmartDashboard.putString("Intake State", "NOPE");
        slamMotor.setIdleMode(IdleMode.kCoast);
        soapBar.setIdleMode(IdleMode.kCoast);
        whooshMotor.setOpenLoopRampRate(0.25);
        slamMotor.setOpenLoopRampRate(0.25);
        intakeMotor.setInverted(true);

        shooterSolenoid.set(Value.kForward);
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
            case PUKE:
                SmartDashboard.putString("Intake State", "Full");
                puke();
                break;
            case REVVING:
                SmartDashboard.putString("Intake State", "REVVING");
                rev();
                break;
            case SHOOTING:
                if (automated)
                    shoot();
                else {
                    shootArbitrary(botArbitrary, topArbitrary);
                }
                SmartDashboard.putString("Intake State", "Shooting");
                break;
            case INDEXING:
                runIndexer();
                SmartDashboard.putString("Intake State", "Indexing");
                break;
        }
        SmartDashboard.putBoolean("running chute?", swallow);
        SmartDashboard.putBoolean("intake up?", intakeSolenoid.get()==IndexerConstants.INTAKE_DEPLOYED);
        SmartDashboard.putBoolean("inner Port?", innerPort);
    }

    /**
     * Turns all indexer motors off
     */

    private boolean heightLimitPassed(){
        if(breakBeam.get() == IndexerConstants.NOT_BROKEN)
            return false;

        else
            return true;
        
     }

    public synchronized void setOff() {
        indexerState = IndexerState.NOPE;
        spit();
    }

    private void runIndexer(){
        sideChew();
          if (!heightLimitPassed()) 
            swallow(-.5);
         else if(heightLimitPassed()){
            swallow(0);
         }
    }
    public synchronized void toggleRPMTolerance() {
        shooter.toggleRPMTolerance();
    }

    public synchronized void setRevving() {
        indexerState = IndexerState.REVVING;
    }

    public synchronized void setIndexing(){
        indexerState = IndexerState.INDEXING;
    }
    
    public synchronized void setInnerPortMode(){
        innerPort = !innerPort;
    }

    public synchronized void rev() {
        ShooterSpeed shot;
        if(innerPort)
        //shot = shotGen.getShot(Turret.getInstance().getInnerPortDist());
            shot = shotGen.getShot(Turret.getInstance().getDistanceToWall());
        
        else
            shot = shotGen.getShot(Turret.getInstance().getDistanceToWall());
            
        shooter.atSpeed(shot.bottomSpeed + botRPMAdjust, shot.topSpeed + topRPMAdjust);
 //shooter.atSpeed(4000, 4000);
    }

    public synchronized void setPuke() {
        indexerState = IndexerState.PUKE;
        vomit();
    }

    public synchronized void setRPMAdjustment(double bot, double top) {
        botRPMAdjust = bot;
        topRPMAdjust = top;
        SmartDashboard.putNumber("botRPMAdjust", botRPMAdjust);
        SmartDashboard.putNumber("topRPMAdjust", topRPMAdjust);
    }

    public synchronized void setRPMRatio(double ratio) {
        rpmRatio = ratio;
        SmartDashboard.putNumber("RPM Ratio", ratio);
    }

    public synchronized double getRPMRatio() {
        return rpmRatio;
    }

    //merely for testing purposes
    public synchronized void toggleManualBeamBreak(){
        passed = !passed;
    }

    public synchronized void setBotRPM(double rpm) {
        botRPMAdjust = rpm;
        SmartDashboard.putNumber("botRPMAdjust", botRPMAdjust);
    }

    public synchronized void setTopRPM(double rpm) {
        topRPMAdjust = rpm;
        SmartDashboard.putNumber("topRPMAdjust", topRPMAdjust);

    }

    /**
     * Sets the intake to fold out and start running inwards. Also runs the indexer conveyors.
     * @param hungry true = run intake
     */
    public synchronized void setHungry(boolean hungry) {
        if(hungry){
            setSalivation(true);
            indexerState = IndexerState.INTAKING;
            
        }else{
            //setSalivation(false);
            //intakeSolenoid.set(Value.kReverse);
            setOff();
        }
    }
    /**
     * Sets if the chute runs while intaking
     */
    public synchronized void setSwallowing(boolean swallowing) {
           swallow = swallowing;
    }

    public synchronized void toggleShooterAngle() {
        if(turretUp) {
          turretUp = false;
          shooterSolenoid.set(Value.kForward);
        }else{
          turretUp = true;
          shooterSolenoid.set(Value.kReverse);
        }
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

    public synchronized void toggleIntaking(){
        if(intake)
            intake = false;
        else if(!intake)
            intake = true;
    }

    /**
     * Sets the conveyor motors (V-shaped) to funnel balls towards the center
     */
    private void chew() {
        slamMotor.set(-0.2);
    }

    public synchronized void sideChew() {
           slamMotor.set(-.7);
           //slamSlave.set(.2);
    }

    private void puke() {
        slamMotor.set(0.7); //check this to make sure it is the correct value
        whooshMotor.set(0.5);
       // if(intakeSolenoid.get() == IndexerConstants.INTAKE_DEPLOYED)
          //  intakeMotor.set(.5);
    }

    /**
     * Stops all motors except shooter
     */
    private void spit() {
        shooter.atSpeed(0, 0);
        slamMotor.set(0);
        whooshMotor.set(0);
        intakeMotor.set(0);
        soapBar.set(0);
    }

    private void vomit() {
        shooter.atSpeed(0, 0);
        //slamLeft.set(0);
        //slamRight.set(0);
        //whooshMotor.set(0);
        intakeMotor.set(0);
        soapBar.set(0);
    }

    private synchronized void setIndexing(boolean swallow){
            if(swallow)
                swallow(-.5);
    }

    /**
     * Runs the chute upwards
     */
    private void swallow(double speed) {
        whooshMotor.set(speed);
        SmartDashboard.putNumber("chute speed", whooshMotor.get());
    }

    public synchronized void ulcer() {
        whooshMotor.set(0);
    }

    /**
     * Runs the soap bar upwards
     */
    private void dropSoap() {
        soapBar.set(-.3);
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
        if (swallow && !heightLimitPassed()) 
            swallow(-.2);
        else if(swallow &&heightLimitPassed()){
            swallow(0);
            intakeMotor.set(-intakeSpeed);
            }
         sideChew();
         intakeMotor.set(intakeSpeed);
         SmartDashboard.putBoolean("swallowing", swallow);
         SmartDashboard.putBoolean("passed?", heightLimitPassed());

        
        //setSwallowing(true);
    }

    /**
     * Puts indexed balls into the chute. Don't run if at least 2 balls aren't indexed.
     */
    private void digest() {
        swallow(-.5); // run chute
        holdSoap();
    }

    public synchronized void setIntakeSpeed(double speed) {
        this.intakeSpeed = speed;
    }

    /**
     * Shoots using auto generated wheel speeds
     */
    public synchronized void setShooting() {
        automated = true;
        indexerState = IndexerState.SHOOTING;
    }

    /**
     * Shoots with arbitrary values
     * @param bot bot wheel speed
     * @param top top wheel speed
     */
    public void setShooting(double bot, double top) {
        automated = false;
        botArbitrary = bot;
        topArbitrary = top;
        indexerState = IndexerState.SHOOTING;
    }

    public void shoot() {
        ShooterSpeed shot = shotGen.getShot(Turret.getInstance().getDistanceToWall());
        boolean atSpeed = shooter.atSpeed(shot.bottomSpeed + botRPMAdjust, shot.topSpeed + topRPMAdjust);
        boolean lemonShot = shooter.lemonShot();
        SmartDashboard.putBoolean("at speed", atSpeed);
        if(shot.bottomSpeed == 0){
            shooter.setSpeed(0, 0);
        }else if (atSpeed) { 
            dropSoap();
            //chew();
            if(shooter.getRPMTolerance() == ShooterConstants.MAX_ALLOWABLE_ERROR_RPM) swallow(-.5);
            else swallow(-1);
        }
        else {
            //shooter.atSpeed(0, 0);
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
        swallow(-.5);
    }

    public void setLemons(int lemons){
        this.lemons = lemons;
    }

    
    public void shootArbitrary(double bottomSpeed, double topSpeed) {
        boolean atSpeed = shooter.atSpeed(bottomSpeed + botRPMAdjust, topSpeed + topRPMAdjust);
        boolean lemonShot = shooter.lemonShot();
        if(bottomSpeed == 0){
            shooter.setSpeed(0, 0);
        }else if (atSpeed) { 
            dropSoap();
            chew();
            swallow(-.5);
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
        swallow(-.5);
    }

    public synchronized int getLemonCount() {
        return lemons;
    }

}