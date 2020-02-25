package frc.robot.utilities;

import edu.wpi.first.wpilibj.SpeedController;

/**
 * Add your docs here.
 */
public interface SmartSpeedController extends SpeedController {
    void setVelocity(double velocityRotationsPerSecond);
    void setVelocity(double velocityRotationsPerSecond, double feedforwardVolts);
    double getMeasuredVelocity();
    double getTargetVelocity();

    void setPosition(double positionRotations);
    void setPosition(double positionRotations, double feedwordwardVolts);
    void setProfiledPosition(double positionRotations);
    double getMeasuredPosition();
    double getTargetPostion();

    void setVoltage(double voltageVolts);
    double getTargetVoltage();
    double getMeasuredVoltage();

    void setCurrent(double currentAmps);
    double getTargetCurrent();
    double getMeasuredCurrent();

    void setEncoderPosition(double positionRotations);
    void resetEncoder();
}