package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.types.LimelightData;

/**
 * https://docs.limelightvision.io/en/latest/networktables_api.html
 */
public class Limelight extends SubsystemBase {
  private final NetworkTableEntry m_targetFoundEntry;
  private final NetworkTableEntry m_xAngleEntry;
  private final NetworkTableEntry m_yAngleEntry;
  private final NetworkTableEntry m_skewEntry;
  private final NetworkTableEntry m_areaEntry;

  private final LimelightData m_data;

  public Limelight(NetworkTable table) {
    m_targetFoundEntry = table.getEntry("ts");
    m_xAngleEntry = table.getEntry("tx");
    m_yAngleEntry = table.getEntry("tx");
    m_skewEntry = table.getEntry("ty");
    m_areaEntry = table.getEntry("ta");

    m_data = new LimelightData();
  }

  @Override
  public void periodic() {
    m_data.setTargetFound(m_targetFoundEntry.getBoolean(false));
    m_data.setHorizontalAngle(m_xAngleEntry.getDouble(0.0));
    m_data.setVerticalAngle(m_yAngleEntry.getDouble(0.0));
    m_data.setSkew(m_skewEntry.getDouble(0.0));
    m_data.setArea(m_areaEntry.getDouble(0.0));
  }

  public LimelightData getData() {
    return m_data;
  }
}
