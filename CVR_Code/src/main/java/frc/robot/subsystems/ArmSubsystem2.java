package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem2 extends SubsystemBase{
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

    private double pulleySetpoint;
    private double telescopeSetpoint;
    private boolean setpointModeOn;

    public ArmSubsystem2() {
        solenoid = new Solenoid(5,PneumaticsModuleType.CTREPCM, 4);
        m_pulleyMotor = new CANSparkMax(Constants.pulleyID, MotorType.kBrushless);
        m_telescopeMotor = new CANSparkMax(Constants.telescopeID, MotorType.kBrushless);
        m_pulleyMotor.setSmartCurrentLimit(1,60);
        m_telescopeMotor.setSmartCurrentLimit(1,60);
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
    // A zero stops moving
    public void raiseArm(double rate) {
        if(rate > 0) {
            pulleySetpoint = 130;
        } else if (rate < 0) {
            pulleySetpoint = 0;
        } else {
            pulleySetpoint = getPulleyPosition();
        }
    }

    // A positive rate extends the telescope.
    // A negative rate retracts the telescope.
    // A zero stops moving
    public void extendTelescope(double rate) {
        if(rate > 0) {
            telescopeSetpoint = 97;
        } else if (rate < 0) {
            telescopeSetpoint = 0;
        } else {
            telescopeSetpoint = getTelescopePosition();
        }
    }
    
    public void toggleClaw() {
        System.out.println("Toggle Claw " + isSolenoidOn);
        setClaw(!isSolenoidOn);
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

    public void setTarget(double pulleySetpoint, double telescopeSetpoint) {
        this.pulleySetpoint = pulleySetpoint;
        this.telescopeSetpoint = telescopeSetpoint;
    }

    @Override
    public void periodic() {
        double pulleyPosition = getPulleyPosition();
        double telescopePosition = getTelescopePosition();
        double pulleySpeedLimited;
        double telescopeSpeedLimited;
        
        if(pulleyPosition > pulleySetpoint) {
            pulleySpeedLimited = -1;
        } else {
            pulleySpeedLimited = 1;
        }
        if(isPulleyPositionAtTarget()) {
            pulleySpeedLimited = 0;
        }

        if(telescopePosition > telescopeSetpoint) {
            telescopeSpeedLimited = -1;
        } else {
            telescopeSpeedLimited = 1;
        }
        if(isTelescopePositionAtTarget()) {
            telescopeSpeedLimited = 0;
        }
        
        if (pulleyPosition>80 && telescopePosition>36){
            if (telescopeSpeedLimited > 0){
                telescopeSpeedLimited = 0;
            }
            if (pulleySpeedLimited < 0) {
                pulleySpeedLimited = 0;
            }
        } 

        if (pulleyPosition<20 && telescopePosition>60){
            if (telescopeSpeedLimited > 0){
                telescopeSpeedLimited = 0;
            }
            if (pulleySpeedLimited > 0) {
                pulleySpeedLimited = 0;
            }
        }
        if (pulleyPosition>130){
            if (pulleySpeedLimited < 0)
            pulleySpeedLimited=0;
        }
        
        if (telescopePosition>97) {
            if (telescopeSpeedLimited>0)
            telescopeSpeedLimited = 0;
        }

        m_pulleyMotor.set(pulleySpeedLimited);
        m_telescopeMotor.set(telescopeSpeedLimited);
        
        SmartDashboard.putNumber("Pulley Position", pulleyPosition);
        SmartDashboard.putNumber("Telescope Position", telescopePosition);      
    }

    private boolean isPulleyPositionAtTarget(){
        double pulleyPosition = getPulleyPosition();
        return Math.abs(pulleySetpoint-pulleyPosition)< 1;
    }
    private boolean isTelescopePositionAtTarget(){
        double telescopePosition = getTelescopePosition();
        return Math.abs(telescopeSetpoint-telescopePosition)< 1;
    }
  }
