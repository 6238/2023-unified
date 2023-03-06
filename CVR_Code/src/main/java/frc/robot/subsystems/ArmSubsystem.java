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

    public ArmSubsystem() {
        Compressor pcmCompressor = new Compressor(5, PneumaticsModuleType.CTREPCM);
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 4);
        m_pulleyMotor = new CANSparkMax(Constants.pulleyID, MotorType.kBrushless);
        m_telescopeMotor = new CANSparkMax(Constants.telescopeID, MotorType.kBrushless);
        m_pulleySpeed = 0;
        m_telescopeSpeed = 0;
        m_pulleyMotor.setSmartCurrentLimit(1,60);
        m_telescopeMotor.setSmartCurrentLimit(1,60);
        isSolenoidOn = false;
        solenoid.set(isSolenoidOn);

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
        /*
        isSolenoidOn = !isSolenoidOn;
        solenoid.set(isSolenoidOn);
        */
        System.out.println("Toggle Claw");
        solenoid.set(false);
    }

    public boolean isTelescopeStalled() {
        return Math.abs(telescopeEncoder.getVelocity()) < 0.1;
    }

    public boolean isPulleyStalled() {
        return Math.abs(pulleyEncoder.getVelocity()) < 0.1;
    }

    @Override
    public void periodic() {
        m_pulleyMotor.set(m_pulleySpeed);
        m_telescopeMotor.set(m_telescopeSpeed);
    }
  
}
