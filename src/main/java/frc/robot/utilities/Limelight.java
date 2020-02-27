package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.types.LimelightData;

/**
 * Limelight utility is responsible for I/O with both Limelight 2+
 * Feeds turret limelight to flywheel/hood/turret and operator shuffleboard
 * Feeds base limelight to intake vision tracking and driver shuffleboard
 */
public class Limelight{
  //Pulls values from network tables
  //TODO: Add code to pull from both limelights
  private NetworkTable m_limelightTable;
  private NetworkTableInstance m_limelightNI = NetworkTableInstance.getDefault();

  //Creates variables to assign
  private double m_horizontalOffset;
  private double m_verticalOffset;
  private double m_area;
  private double m_targetDistance;

  private boolean m_targetFound;

  //Enum for the two limelights on the robot
  public enum Limelights{
    DRIVER, TURRET;
  }

  public Limelight(Limelights camera) {
    switch(camera){
      case DRIVER:
      m_limelightTable = m_limelightNI.getTable(LimelightConstants.kDriverLimelight);
      break;

      case TURRET:
      m_limelightTable = m_limelightNI.getTable(LimelightConstants.kTurretLimelight);
      break;

      default:
      break;
    }
  }

  public LimelightData getLimelightData(){
    updateReadings();
    return new LimelightData(m_horizontalOffset, m_verticalOffset, m_targetDistance, m_targetFound);
  }

  public void updateReadings(){
    m_horizontalOffset = getHorizontalOffset();
    m_verticalOffset = getVerticalOffset();
    m_targetDistance = getTargetDistance();
    m_targetFound = isTargetFound();
  }

  public double getTargetDistance(){
    double h = (LimelightConstants.kHighGoalHeight - LimelightConstants.kHighLimelightHeight) / 12;
    return h/Math.tan(Math.toRadians(getVerticalOffset() + LimelightConstants.kHighLimelightMountAngle));
  }

  public double getHorizontalOffset(){
    m_horizontalOffset = m_limelightTable.getEntry("tx").getDouble(0);
    return m_horizontalOffset;
  }

  public double getVerticalOffset(){
    m_verticalOffset = m_limelightTable.getEntry("ty").getDouble(0);
    return m_verticalOffset;
  }

  public double getArea(){
    return m_area;
  }

  public boolean isTargetFound(){
    if (m_limelightTable.getEntry("tv").getDouble(0) == 0){
      m_targetFound = false;
    }
    else{
      m_targetFound = true;
    }
    return m_targetFound;
  }
}
