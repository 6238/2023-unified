package frc.robot.commands;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;

public class ArmManualCommand extends CommandBase {
    ArmSubsystem armSubsystem;
    Joystick joystick;

    public ArmManualCommand(ArmSubsystem armSubsystem, Joystick joystick) {
        this.armSubsystem = armSubsystem;
        this.joystick = joystick;

        addRequirements(armSubsystem);
    }

    @Override
    public void execute() {
        boolean raise = joystick.getRawButton(Constants.raiseArmBttn);
        boolean lower = joystick.getRawButton(Constants.lowerArmBttn);
        boolean extend = joystick.getRawButton(Constants.extendArmBttn);
        boolean retract = joystick.getRawButton(Constants.retractArmBttn);

        int raiseRate = 0;
        int extendRate = 0;

        
        if(raise && !lower) {
            raiseRate = 1;
        } else if(!raise && lower) {
            raiseRate = -1;
        } else {
            raiseRate = 0;
        }

        if(extend && !retract) {
            extendRate = 1;
        } else if(!extend && retract) {
            extendRate = -1;
        } else {
            extendRate = 0;
        }

        armSubsystem.raiseArm(raiseRate);
        armSubsystem.extendTelescope(extendRate);
    }

    @Override
    public void end(boolean interrupt) {
        armSubsystem.resetPulley();
        armSubsystem.resetTelescope();
    }

    // 1st is rotation, 2nd is extension
    private Pair<Double,Double> up(int y) {
        double e = (armSubsystem.getTelescopePosition()-77)*Constants.telescopeMetersPerCount + Constants.armLengthAtGrab;
        double theta = -(armSubsystem.getPulleyPosition()-81)*Constants.armThetaPerCount + Constants.thetaAtGrab;
        double extension = Math.sqrt(e + 2*e*Math.sin(theta)*y + y*y) - e; // Meters
        double raise = 180 * Math.atan((Math.sin(theta) + y/e)/Math.cos(theta)) / Math.PI; // Degrees

        extension = extension * Constants.telescopeGearRatio / Constants.telescopeMetersPerCount;
        raise = raise * Constants.pulleyGearRatio / Constants.armThetaPerCount;

        return new Pair<Double,Double>(raise, extension);
    }

    // 1st is rotation, 2nd is extension
    private Pair<Double,Double> forward(int x) {
        double e = (armSubsystem.getTelescopePosition()-77)*Constants.telescopeMetersPerCount + Constants.armLengthAtGrab;
        double theta = -(armSubsystem.getPulleyPosition()-81)*Constants.armThetaPerCount + Constants.thetaAtGrab;
        double extension = Math.sqrt(e + 2*e*Math.cos(theta)*x + x*x) - e; // Meters
        double raise = 180 * Math.atan(Math.sin(theta)/(Math.cos(theta) + x/e)) / Math.PI; // Degrees

        extension = extension * Constants.telescopeGearRatio / Constants.telescopeMetersPerCount;
        raise = raise * Constants.pulleyGearRatio / Constants.armThetaPerCount;

        return new Pair<Double,Double>(raise, extension);
    }
}
