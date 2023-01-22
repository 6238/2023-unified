package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.DriveSubsystem;

public class BalanceCommand extends PIDCommand {
    public final double kTurnToleranceDeg = 2;
    public final double kTurnRateToleranceDegPerS = 1;
    
    double balanceKP = 0;
    double balanceKI = 0;
    double balanceKD = 0;

    public BalanceCommand(DriveSubsystem driveSubsystem) {
        super (
            new PIDController(3, 0, 0),
            driveSubsystem::getPitch,
            0,
            output -> driveSubsystem.arcadeDrive(output, 0),
            driveSubsystem
        );
        /*
        //------------------------------------------------------------------------
        SmartDashboard.putNumber("balanceKp",
            SmartDashboard.getNumber("balanceKp", 0));
        SmartDashboard.putNumber("balanceKi",
            SmartDashboard.getNumber("balanceKi", 0));
        SmartDashboard.putNumber("balanceKd",
            SmartDashboard.getNumber("balanceKd", 0));

        balanceKP = SmartDashboard.getNumber("balanceKp", 0);
        balanceKI = SmartDashboard.getNumber("balanceKi", 0);
        balanceKD = SmartDashboard.getNumber("balanceKi", 0);

        getController().setP(balanceKP);
        getController().setI(balanceKI);
        getController().setD(balanceKD);
        //------------------------------------------------------------------------
        */
        getController().enableContinuousInput(-180, 180);

        getController()
        .setTolerance(kTurnToleranceDeg, kTurnRateToleranceDegPerS);

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        /*
        double kP = SmartDashboard.getNumber("balanceKp", 0);
        double kI = SmartDashboard.getNumber("balanceKi", 0);
        double kD = SmartDashboard.getNumber("balanceKi", 0);

        if (balanceKP != kP) {
            getController().setP(balanceKP);
            balanceKP = kP;
        }
        if (balanceKI == kI) {
            getController().setI(balanceKI);
            balanceKI = kI;
        }
        if (balanceKD == kD) {
            getController().setD(balanceKD);
            balanceKD = kD;
        }
        */
    }

    // @Override
    // public boolean isFinished() {
    //     // End when the controller is at the reference.
    //     return getController().atSetpoint();
    // }
}
