package frc.org.ballardrobotics.speedcontrollers.ctre;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.org.ballardrobotics.speedcontrollers.SmartSpeedController;

/**
 * Add your docs here.
 */
public class SmartTalonSRX extends WPI_TalonSRX implements SmartSpeedController {
    public static final double kNominalVoltageVolts = 12.0;
    public static final int kVelocitySlotIdx = 0;
    public static final int kPositionSlotIdx = 1;

    private final double kUnitsPerRevolution;

    private ControlMode m_lastControlMode = ControlMode.Disabled;
    private DemandType m_lastDemand1Type = DemandType.Neutral;
    private double m_lastDemand0, m_lastDemand1;

    private double m_targetVelocityRPM;
    private double m_targetPositionRevolutions;
    private double m_targetVoltageVolts;
    private double m_targetCurrentAmps;

    public SmartTalonSRX(int deviceNumber) {
        this(deviceNumber, 1.0);
    }

    public SmartTalonSRX(int deviceNumber, double unitsPerRevolution) {
        super(deviceNumber);
        kUnitsPerRevolution = unitsPerRevolution;
        configVoltageCompSaturation(kNominalVoltageVolts);
        enableVoltageCompensation(true);
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
        set(ControlMode.Velocity, (velocityRPM * kUnitsPerRevolution) / (60.0 * 10.0), DemandType.ArbitraryFeedForward,
                feedforwardVolts / kNominalVoltageVolts);
    }

    @Override
    public double getMeasuredVelocity() {
        return (getSelectedSensorVelocity() / kUnitsPerRevolution) * (60.0 * 10.0);
    }

    @Override
    public double getTargetVelocity() {
        return m_targetVelocityRPM;
    }

    @Override
    public void setPosition(double position) {
        m_targetPositionRevolutions = position;
        set(ControlMode.Position, position * kUnitsPerRevolution);
    }

    @Override
    public double getMeasuredPosition() {
        return (getSelectedSensorPosition() / kUnitsPerRevolution);
    }

    @Override
    public double getTargetPostion() {
        return m_targetPositionRevolutions;
    }

    @Override
    public void setVoltage(double voltageVolts) {
        m_targetVoltageVolts = voltageVolts;
        set(ControlMode.PercentOutput, voltageVolts / kNominalVoltageVolts);
    }

    @Override
    public double getTargetVoltage() {
        return m_targetVoltageVolts;
    }

    @Override
    public double getMeasuredVoltage() {
        return getMotorOutputVoltage();
    }

    @Override
    public void setCurrent(double currentAmps) {
        m_targetCurrentAmps = currentAmps;
        set(ControlMode.Current, currentAmps);
    }

    @Override
    public double getTargetCurrent() {
        return m_targetCurrentAmps;
    }

    @Override
    public double getMeasuredCurrent() {
        return getSupplyCurrent();
    }

    @Override
    public void set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) {
        if (mode != m_lastControlMode || demand0 != m_lastDemand0 || demand1Type != m_lastDemand1Type
                || m_lastDemand1 != demand1) {
            m_lastControlMode = mode;
            m_lastDemand0 = demand0;
            m_lastDemand1Type = demand1Type;
            m_lastDemand1 = demand1;

            super.set(mode, demand0, demand1Type, demand1);
        } else {
            // Always feed the watchdog.
            feed();
        }
    }
}
