package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem3 extends SubsystemBase{
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

    public ArmSubsystem3() {
        solenoid = new Solenoid(5,PneumaticsModuleType.CTREPCM, 4);
        m_pulleyMotor = new CANSparkMax(Constants.pulleyID, MotorType.kBrushless);
        m_telescopeMotor = new CANSparkMax(Constants.telescopeID, MotorType.kBrushless);
        m_pulleySpeed = 0;
        m_telescopeSpeed = 0;
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

    public void deactivateSetpointMode() {
        this.setpointModeOn = false;
    }

    public void activateSetpointMode(double pulleySetpoint, double telescopeSetpoint) {
        this.setpointModeOn = true;
        this.pulleySetpoint = pulleySetpoint;
        this.telescopeSetpoint = telescopeSetpoint;
    }

    @Override
    public void periodic() {
        double pulleyPosition = getPulleyPosition();
        double telescopePosition = getTelescopePosition();
        double pulleySpeedLimited = m_pulleySpeed;
        double telescopeSpeedLimited = m_telescopeSpeed;
        
        if(setpointModeOn) {
            System.out.println("Inside Threshold: " + isPulleyPositionAtTarget() + " Pulley Speed: " + pulleySpeedLimited);
            if (isPulleyPositionAtTarget()){
                pulleySpeedLimited = 0;
            } else if (pulleySetpoint < pulleyPosition){
                pulleySpeedLimited = 1;
            } else if (pulleySetpoint > pulleyPosition){
                pulleySpeedLimited = -1;
            }
            if (isTelescopePositionAtTarget()) {
                telescopeSpeedLimited =0;
            } else if (telescopeSetpoint<telescopePosition){
                telescopeSpeedLimited = -1;
            } else if (telescopeSetpoint>telescopePosition){
                telescopeSpeedLimited = 1;
            }
        } else {
            System.out.println("Setpoint Mode Off");
        }

        if (pulleyPosition>80 / 3 && telescopePosition>36 / 3){
            if (telescopeSpeedLimited > 0){
                telescopeSpeedLimited = 0;
            }
            if (pulleySpeedLimited < 0) {
                pulleySpeedLimited = 0;
            }
        } 

        if (pulleyPosition<20 / 3 && telescopePosition>60 / 3){
            if (telescopeSpeedLimited > 0){
                telescopeSpeedLimited = 0;
            }
            if (pulleySpeedLimited > 0) {
                pulleySpeedLimited = 0;
            }
        }
        if (pulleyPosition>130 / 3){
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
