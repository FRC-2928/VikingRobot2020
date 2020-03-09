package org.ballardrobotics.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Twist2d;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Subsystems must implement all of these methods.
 */
public interface SmartSubsystem extends Subsystem {

    // Control Inputs
    public void setPositionReference(Pose2d reference);
    public void setPosition();
    public void setVelocity(Twist2d velocity);

    // System State
    public Pose2d getPosition();
    public Twist2d getVelocity();
    public boolean atReference();
}    