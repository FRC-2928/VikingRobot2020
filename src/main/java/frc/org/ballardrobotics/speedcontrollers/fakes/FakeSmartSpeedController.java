package frc.org.ballardrobotics.speedcontrollers.fakes;

import java.util.function.DoubleSupplier;

import frc.org.ballardrobotics.speedcontrollers.SmartSpeedController;

/**
 * Add your docs here.
 */
public class FakeSmartSpeedController implements SmartSpeedController {
    public static final double kNominalVoltageVolts = 12.0;

    private double m_percentOutput, m_voltage, m_current, m_position, m_velocity, m_feedforward;
    private boolean m_isInverted, m_disabled;

    public final DoubleSupplier kDefaultMeasuredVoltageSupplier = () -> m_voltage;
    public final DoubleSupplier kDefaultMeasuredCurrentSupplier = () -> m_current;
    public final DoubleSupplier kDefaultMeasuredPositionSupplier = () -> m_position;
    public final DoubleSupplier kDefaultMeasuredVelocitySupplier = () -> m_velocity;
    
    private DoubleSupplier m_measuredVoltageSupplier = kDefaultMeasuredVoltageSupplier;
    private DoubleSupplier m_measuredCurrentSupplier = kDefaultMeasuredCurrentSupplier;
    private DoubleSupplier m_measuredPositionSupplier = kDefaultMeasuredPositionSupplier;
    private DoubleSupplier m_measuredVelocitySupplier = kDefaultMeasuredVelocitySupplier;

    @Override
    public void set(double speed) {
        m_percentOutput = speed;
        setVoltage(speed * kNominalVoltageVolts);
    }

    @Override
    public double get() {
        return m_percentOutput;
    }

    @Override
    public void setInverted(boolean isInverted) {
        m_isInverted = isInverted;
    }

    @Override
    public boolean getInverted() {
        return m_isInverted;
    }

    @Override
    public void disable() {
        m_disabled = true;
    }

    @Override
    public void stopMotor() {
        m_percentOutput = 0.0;
        m_voltage = 0.0;
        m_current = 0.0;
        m_velocity = 0.0;
    }

    @Override
    public void pidWrite(double output) {
        m_percentOutput = output;
    }

    @Override
    public void setVelocity(double velocityRPM) {
        setVelocity(velocityRPM, 0.0);
    }

    @Override
    public void setVelocity(double velocityRPM, double feedforwardVolts) {
        m_velocity = velocityRPM;
        m_feedforward = feedforwardVolts;
    }

    @Override
    public double getMeasuredVelocity() {
        return m_measuredVelocitySupplier.getAsDouble();
    }

    @Override
    public double getTargetVelocity() {
        return m_velocity;
    }

    @Override
    public void setPosition(double positionRotations) {
        m_position = positionRotations;
    }

    @Override
    public double getMeasuredPosition() {
        return m_measuredPositionSupplier.getAsDouble();
    }

    @Override
    public double getTargetPostion() {
        return m_position;
    }

    @Override
    public void setVoltage(double voltageVolts) {
        m_voltage = voltageVolts;
    }

    @Override
    public double getTargetVoltage() {
        return m_voltage;
    }

    @Override
    public double getMeasuredVoltage() {
        return m_measuredVoltageSupplier.getAsDouble();
    }

    @Override
    public void setCurrent(double currentAmps) {
        m_current = currentAmps;
    }

    @Override
    public double getTargetCurrent() {
        return m_current;
    }

    @Override
    public double getMeasuredCurrent() {
        return m_measuredCurrentSupplier.getAsDouble();
    }

    @Override
    public void resetEncoder() {
        m_position = 0;
    }

    public double getFeedforward() {
        return m_feedforward;
    }

    public boolean getDisabled() {
        return m_disabled;
    }

    public void setMeasuredVoltageSupplier(DoubleSupplier measuredVoltageSupplier) {
        m_measuredVoltageSupplier = measuredVoltageSupplier;
    }

    public void setMeasuredCurrentSupplier(DoubleSupplier measuredCurrentSupplier) {
        m_measuredCurrentSupplier = measuredCurrentSupplier;
    }

    public void setMeasuredPositionSupplier(DoubleSupplier measuredPositionSupplier) {
        m_measuredPositionSupplier = measuredPositionSupplier;
    }

    public void setMeasuredVelocitySupplier(DoubleSupplier measuredVelocitySupplier) {
        m_measuredVelocitySupplier = measuredVelocitySupplier;
    }
}
