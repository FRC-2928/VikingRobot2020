package frc.org.ballardrobotics.speedcontrollers.rev;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.EncoderType;

import frc.org.ballardrobotics.speedcontrollers.SmartSpeedController;

/**
 * Add your docs here.
 */
public class SmartSparkMax extends CANSparkMax implements SmartSpeedController {
    public static final double kNominalVoltageVolts = 12.0;
    public static final int kVelocitySlotIdx = 0;
    public static final int kPositionSlotIdx = 1;

    private double m_targetVelocityRPM;
    private double m_targetPositionRevolutions;
    private double m_targetVoltageVolts;
    private double m_targetCurrentAmps;

    private CANEncoder m_encoder;
    private CANPIDController m_pidController;

    public SmartSparkMax(int deviceID, MotorType type) {
        this(deviceID, type, 1.0);
    }

    public SmartSparkMax(int deviceID, MotorType type, double gearRatio) {
        this(deviceID, type, EncoderType.kHallSensor, 0, gearRatio);
    }

    public SmartSparkMax(int deviceID, MotorType type, EncoderType encoderType, double unitsPerRevolution, double gearRatio) {
        super(deviceID, type);
        enableVoltageCompensation(kNominalVoltageVolts);

        m_encoder = new CANEncoder(this, encoderType, (int)unitsPerRevolution);
        m_pidController = super.getPIDController();

        m_encoder.setPositionConversionFactor(1.0 / gearRatio);
        m_encoder.setVelocityConversionFactor(1.0 / gearRatio);
    }

    @Override
    public void set(double value) {
        setVoltage(value * kNominalVoltageVolts);
    }

    @Override
    public void setVelocity(double velocityRPM) {
        setVelocity(velocityRPM, 0.0);
    }

    @Override
    public void setVelocity(double velocityRPM, double feedforwardVolts) {
        m_targetVelocityRPM = velocityRPM;
        m_pidController.setReference(velocityRPM, ControlType.kVelocity, kVelocitySlotIdx, feedforwardVolts, ArbFFUnits.kVoltage);
    }

    @Override
    public double getMeasuredVelocity() {
        return m_encoder.getVelocity();
    }

    @Override
    public double getTargetVelocity() {
        return m_targetVelocityRPM;
    }

    @Override
    public void setPosition(double positionRotations) {
        m_targetPositionRevolutions = positionRotations;
        m_pidController.setReference(positionRotations, ControlType.kPosition, kPositionSlotIdx);
    }

    @Override
    public double getMeasuredPosition() {
        return m_encoder.getPosition();
    }

    @Override
    public double getTargetPostion() {
        return m_targetPositionRevolutions;
    }

    @Override
    public void setVoltage(double voltageVolts) {
        m_targetVoltageVolts = voltageVolts;
        super.setVoltage(voltageVolts);
    }

    @Override
    public double getTargetVoltage() {
        return m_targetVoltageVolts;
    }

    @Override
    public double getMeasuredVoltage() {
        return getBusVoltage() * getAppliedOutput();
    }

    @Override
    public void setCurrent(double currentAmps) {
        m_targetCurrentAmps = currentAmps;
        m_pidController.setReference(currentAmps, ControlType.kCurrent);
    }

    @Override
    public double getTargetCurrent() {
        return m_targetCurrentAmps;
    }

    @Override
    public double getMeasuredCurrent() {
        return getOutputCurrent();
    }

    @Override
    public void resetEncoder() {
        m_encoder.setPosition(0.0);
    }

    @Override
    public CANPIDController getPIDController() {
        return m_pidController;
    }
}
