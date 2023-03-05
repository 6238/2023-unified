package frc.robot.subsystems;

import java.util.concurrent.ThreadPoolExecutor.CallerRunsPolicy;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
    private final Solenoid solenoid;
    private final CANSparkMax m_pulleyMotor;
    private final CANSparkMax m_telescopeMotor;

    public ArmSubsystem() {
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        m_pulleyMotor = new CANSparkMax(Constants.pulleyID, MotorType.kBrushless);
        m_telescopeMotor = new CANSparkMax(Constants.telescopeID, MotorType.kBrushless);
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

    public void reset() {
        m_pulleyMotor.set(0);
        m_telescopeMotor.set(0);
    }

    public void extendSolenoid() {
        solenoid.set(true);
    }

    public void retractSolenoid() {
        solenoid.set(false);
    }
}
