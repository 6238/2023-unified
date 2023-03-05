package frc.robot.subsystems;


import java.util.concurrent.ThreadPoolExecutor.CallerRunsPolicy;

import com.ctre.phoenix.music.*;
import com.kauailabs.navx.*;
import com.revrobotics.*;
import org.photonvision.PhotonCamera;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
    private final Solenoid solenoid;
    private final CANSparkMax m_pulleyMotor;
    private final CANSparkMax m_telescopeMotor;

    private boolean isSolenoidOn;

    public ArmSubsystem() {
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        m_pulleyMotor = new CANSparkMax(Constants.pulleyID, MotorType.kBrushless);
        m_telescopeMotor = new CANSparkMax(Constants.telescopeID, MotorType.kBrushless);

        isSolenoidOn = false;
        solenoid.set(isSolenoidOn);
    }

    // A positive rate raises the arm.
    // A negative rate lowers the arm.
    public void raiseArm(double rate) {
        rate = Math.abs(rate) <= 0.5 ? rate : 0.5 * rate / Math.abs(rate); 
        m_pulleyMotor.set(rate);
    }


    // A positive rate extends the telescope.
    // A negative rate retracts the telescope.
    public void extendTelescope(double rate) {
        rate = Math.abs(rate) <= 0.5 ? rate : 0.5 * rate / Math.abs(rate);
        m_telescopeMotor.set(rate);
    }

    public void resetPulley() {
        m_pulleyMotor.set(0);
    }

    public void resetTelescope() {
        m_telescopeMotor.set(0);
    }
    
    public void toggleClaw() {
        isSolenoidOn = !isSolenoidOn;
        solenoid.set(isSolenoidOn);
    }
}