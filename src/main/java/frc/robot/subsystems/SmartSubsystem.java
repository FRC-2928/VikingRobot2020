package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Subsystems must implement all of these methods.
 */
public interface SmartSubsystem extends Subsystem {

    // Control Inputs
    public void setPower(double power);
    public void setPosition(double position);
    public void setVelocity(double velocity);
    public void setMotion(double position);
    public void stop();

    // System State
    public double getPosition();
    public double getVelocity();
    public boolean atReference();
}    