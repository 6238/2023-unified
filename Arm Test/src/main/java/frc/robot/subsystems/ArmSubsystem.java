package frc.robot.subsystems;

import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase{
    private final Solenoid solenoid;
    private final CANSparkMax m_pulleyMotor;

    public ArmSubsystem() {
        solenoid = null; //new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        m_pulleyMotor = new CANSparkMax(Constants.SparkMaxID, MotorType.kBrushless);
    }

    public void raiseArm(double rate) {
        if(rate > 0.5) {
            rate = 0.5;
        }
        m_pulleyMotor.set(rate);
        SmartDashboard.putNumber("Motor Current", m_pulleyMotor.getOutputCurrent());
    }

    public void reset() {
        m_pulleyMotor.set(0);
    }

    public void extendSolenoid() {
        System.out.println("Running Extend Solenoid.");
        solenoid.set(true);
    }

    public void retractSolenoid() {
        System.out.println("Running Retract Solenoid.");
        solenoid.set(false);
    }
}
