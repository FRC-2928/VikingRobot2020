package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Limelight takes readings from the two limelight cameras and feeds it to the turret and shooter
 */
public class Limelight extends SubsystemBase {

  //Pulls values from network tables
  //TODO: Add code to pull from both limelights
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  //Creates variables to assign
  double x;
  double y;
  double area;

  public Limelight() {

  }

  @Override
  public void periodic() {
    updateReadings();
  }

  public void updateReadings(){
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
  }
}
