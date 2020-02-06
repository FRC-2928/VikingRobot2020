package frc.robot.types;

public class ShooterSetpoint {
    public double hoodPositionDegrees;
    public double turretPositionDegrees;
    public double flywheelVelocityRPM;

    public ShooterSetpoint(double hoodPositionDegrees, double turretPositionDegrees, double flywheelVelocityRPM) {
        this.hoodPositionDegrees = hoodPositionDegrees;
        this.turretPositionDegrees = turretPositionDegrees;
        this.flywheelVelocityRPM = flywheelVelocityRPM;
    }
}
