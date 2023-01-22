package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SmartDashboardParam {
    private final String name;
    public SmartDashboardParam (String name, double defaultValue) {
        this.name = name;
        double uiValue = SmartDashboard.getNumber(this.name, defaultValue);
        SmartDashboard.putNumber(this.name, uiValue);
    }

    public double get() {
        return SmartDashboard.getNumber(name, 0);
    }
}