package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.MathUtil;

public class ArmSubsystem extends SubsystemBase{
    private final Solenoid solenoid;
    private final CANSparkMax m_pulleyMotor;
    private final CANSparkMax m_telescopeMotor;
    private double m_pulleySpeed;
    private double m_telescopeSpeed;    
    private boolean isSolenoidOn;

    private final RelativeEncoder telescopeEncoder;
    private final RelativeEncoder pulleyEncoder;
    private double m_pulleyPositionHome;
    private double m_telescopePostionHome;

    private int timer = 0;

    private DigitalInput IRSensor = new DigitalInput(9);
    private double pulleySetpoint;
    private double telescopeSetpoint;

    private boolean setpointModeOnPulley;
    private boolean setpointModeOnTelescope;

    public ArmSubsystem() {
        solenoid = new Solenoid(5,PneumaticsModuleType.CTREPCM, 4);
        m_pulleyMotor = new CANSparkMax(Constants.pulleyID, MotorType.kBrushless);
        m_telescopeMotor = new CANSparkMax(Constants.telescopeID, MotorType.kBrushless);
        m_pulleySpeed = 0;
        m_telescopeSpeed = 0;
        m_pulleyMotor.setSmartCurrentLimit(2,4, 10);
        m_telescopeMotor.setSmartCurrentLimit(2,4, 10);
        isSolenoidOn = false;
        solenoid.set(isSolenoidOn);
        m_pulleyPositionHome = 0;
        m_telescopePostionHome = 0;

        pulleyEncoder = m_pulleyMotor.getEncoder();
        telescopeEncoder = m_telescopeMotor.getEncoder();

        pulleyEncoder.setVelocityConversionFactor(1.0);
        telescopeEncoder.setVelocityConversionFactor(1.0);
        pulleyEncoder.setPositionConversionFactor(1.0);
        telescopeEncoder.setPositionConversionFactor(1.0);
        pulleyEncoder.setMeasurementPeriod(32);
        telescopeEncoder.setMeasurementPeriod(32);
        pulleyEncoder.setAverageDepth(8);
        telescopeEncoder.setAverageDepth(8);
    }

    // A positive rate raises the arm.
    // A negative rate lowers the arm.
    public void raiseArm(double rate) {
        m_pulleySpeed = rate;
    }

    // A positive rate extends the telescope.
    // A negative rate retracts the telescope.
    public void extendTelescope(double rate) {
        m_telescopeSpeed = rate;
    }

    public void resetPulley() {
        m_pulleySpeed = 0;
    }

    public void resetTelescope() {
        m_telescopeSpeed = 0;
    }
    
    public void toggleClaw() {
        System.out.println("Toggle Claw " + isSolenoidOn);
        if(!isSolenoidOn) timer = 0;
        isSolenoidOn = !isSolenoidOn;
        solenoid.set(isSolenoidOn);
    }

    public void setClaw(boolean open) {
        isSolenoidOn = open;
        solenoid.set(isSolenoidOn);
    }

    public double getPulleyPosition(){
        return -pulleyEncoder.getPosition()+m_pulleyPositionHome;
    }

    public double getTelescopePosition(){
        return telescopeEncoder.getPosition()-m_telescopePostionHome;
    }

    public boolean isTelescopeStalled() {
        return Math.abs(telescopeEncoder.getVelocity()) < 0.1;
    }

    public boolean isPulleyStalled() {
        return Math.abs(pulleyEncoder.getVelocity()) < 0.1;
    }

    public void resetHome() {
        m_pulleyPositionHome = pulleyEncoder.getPosition();
        m_telescopePostionHome = telescopeEncoder.getPosition();
    }


    public void deactivatePulleySetpoint() {
        this.setpointModeOnPulley = false;
    }

    public void deactivateTelescopeSetpoint() {
        this.setpointModeOnTelescope = false;
    }

    public void activateSetpointModePulley(double pulleySetpoint) {
        this.setpointModeOnPulley = true;
        this.pulleySetpoint = pulleySetpoint;
    }

    public void activateSetpointModeTelescope(double telescopeSetpoint) {
        this.setpointModeOnTelescope = true;
        this.telescopeSetpoint = telescopeSetpoint;
    }

    public void activateSetpointMode(double pulleySetpoint, double telescopeSetpoint) {
        this.setpointModeOnTelescope = true;
        this.setpointModeOnPulley = true;
        this.pulleySetpoint = pulleySetpoint;
        this.telescopeSetpoint = telescopeSetpoint;
    }

    @Override
    public void periodic() {
        double pulleyPosition = getPulleyPosition();
        double telescopePosition = getTelescopePosition();
        double pulleySpeedLimited = m_pulleySpeed;
        double telescopeSpeedLimited = m_telescopeSpeed;
        
        if(setpointModeOnPulley) {
            if (isPulleyPositionAtTarget()){
                double error = pulleySetpoint - pulleyPosition;
                pulleySpeedLimited = -MathUtil.scaleMagnitude(error, 0, 2.0, 0.0, 0.2, 1.0);
                //pulleySpeedLimited -= 0.2;
            } 
            else if (pulleySetpoint < pulleyPosition){
                pulleySpeedLimited = 1;
            } else if (pulleySetpoint > pulleyPosition){
                pulleySpeedLimited = -1;
            }
        }

        if (setpointModeOnTelescope) {
            if (isTelescopePositionAtTarget()) {
                telescopeSpeedLimited = 0;
            } else if (telescopeSetpoint<telescopePosition){
                telescopeSpeedLimited = -1;
            } else if (telescopeSetpoint>telescopePosition){
                telescopeSpeedLimited = 1;
            }
        }

        if (pulleyPosition>40.0 && telescopePosition > 10.0){
            if (telescopeSpeedLimited > 0){
                telescopeSpeedLimited = 0;
            }
            if (pulleySpeedLimited < 0) {
                pulleySpeedLimited = 0;
            }
        } 

        if (pulleyPosition < 15 && telescopePosition> 34){
            if (telescopeSpeedLimited > 0){
                telescopeSpeedLimited = 0;
            }
            if (pulleySpeedLimited > 0) {
                pulleySpeedLimited = 0;
            }
        }

        if (pulleyPosition> 55){
            if (pulleySpeedLimited < 0)
            pulleySpeedLimited=0;
        }

        if (telescopePosition>41.0) {
            if (telescopeSpeedLimited>0)
            telescopeSpeedLimited = 0;
        }

        pulleySpeedLimited = edu.wpi.first.math.MathUtil.clamp(pulleySpeedLimited, -0.4, 0.7);
        m_pulleyMotor.set(pulleySpeedLimited);
        m_telescopeMotor.set(telescopeSpeedLimited);

        if(timer < 80) timer++;
        SmartDashboard.putBoolean("IR Sensor", !IRSensor.get());
        if(!IRSensor.get() && isSolenoidOn && timer == 80) {
            setClaw(false);
        }
        
        SmartDashboard.putNumber("Pulley Position", pulleyPosition);
        SmartDashboard.putNumber("Telescope Position", telescopePosition);     
    }

    // returns 1 if the pulley is allowed to move down
    public boolean canPulleyMoveDown() {
        return !(getPulleyPosition() > 35.0 && getTelescopePosition() > 10.0);
    }

    private boolean isPulleyPositionAtTarget(){
        double pulleyPosition = getPulleyPosition();
        return Math.abs(pulleySetpoint-pulleyPosition)< 2.0;
    }
    private boolean isTelescopePositionAtTarget(){
        double telescopePosition = getTelescopePosition();
        return Math.abs(telescopeSetpoint-telescopePosition)< 1.0;
    }
  }
