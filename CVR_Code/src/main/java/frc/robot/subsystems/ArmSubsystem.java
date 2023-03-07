package frc.robot.subsystems;

import java.util.ArrayDeque;
import java.util.Deque;
import java.util.concurrent.ThreadPoolExecutor.CallerRunsPolicy;

import com.ctre.phoenix.music.*;
import com.kauailabs.navx.*;
import com.revrobotics.*;
import org.photonvision.PhotonCamera;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    public ArmSubsystem() {
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
        isSolenoidOn = !isSolenoidOn;
        solenoid.set(isSolenoidOn);
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


    @Override
    public void periodic() {
        double pulleyPosition = -pulleyEncoder.getPosition()+m_pulleyPositionHome;
        double telescopePosition = telescopeEncoder.getPosition()-m_telescopePostionHome;
        double pulleySpeedLimited = m_pulleySpeed;
        double telescopeSpeedLimited = m_telescopeSpeed;
        
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

        m_pulleyMotor.set(pulleySpeedLimited);
        m_telescopeMotor.set(telescopeSpeedLimited);
        
        SmartDashboard.putNumber("Pulley Position", pulleyPosition);
        SmartDashboard.putNumber("Telescope Position", telescopePosition);
        
    }
  
    }
