package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private final DoubleSolenoid solenoid;
    private final CANSparkMax m_pulleyMotor, m_telescopeMotor;

    public ArmSubsystem() {
        solenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 6, 7);
        m_pulleyMotor = new CANSparkMax(Constants.PulleyID, MotorType.kBrushless);
        m_telescopeMotor = new CANSparkMax(Constants.TelescopeID, MotorType.kBrushless);
    }

    public void raiseArm(double rate) {
        if(rate > 0.5) {
            rate = 0.5;
        }
        m_pulleyMotor.set(rate);
    }

    public void extendArm(double rate) {
        if(rate > 0.5) {
            rate = 0.5;
        }
        m_telescopeMotor.set(rate);
    }

    public void resetPulley() {
        m_pulleyMotor.set(0);
    }

    public void resetTelescope() {
        m_telescopeMotor.set(0);
    }

    public void toggleClaw() {
        if(solenoid.get() == Value.kOff) {
            solenoid.set(Value.kForward);
        } else {
            solenoid.toggle();
        }
    }
}
