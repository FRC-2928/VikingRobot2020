package frc.org.ballardrobotics.speedcontrollers;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Add your docs here.
 */
public interface SmartSpeedController extends SpeedController {
    void setVelocity(double velocityRPM);
    void setVelocity(double velocityRPM, double feedforwardVolts);
    double getMeasuredVelocity();
    double getTargetVelocity();

    void setPosition(double positionRotations);
    double getMeasuredPosition();
    double getTargetPostion();

    void setVoltage(double voltageVolts);
    double getTargetVoltage();
    double getMeasuredVoltage();

    void setCurrent(double currentAmps);
    double getTargetCurrent();
    double getMeasuredCurrent();
}
