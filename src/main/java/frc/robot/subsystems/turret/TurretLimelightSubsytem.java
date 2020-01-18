package frc.robot.subsystems.turret;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
/**
 * TurretLimelightSubsytem takes input from the limelight camera
 * Also responsible for doing angle and distance calculations
 */
public class TurretLimelightSubsytem extends SubsystemBase {
  //Pulls values from network tables
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  //Creates variables to assign
  double x;
  double y;
  double area;

  public TurretLimelightSubsytem() {
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
