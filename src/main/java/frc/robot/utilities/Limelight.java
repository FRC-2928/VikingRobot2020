package frc.robot.utilities;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Limelight utility is responsible for I/O with both Limelight 2+
 * Feeds turret limelight to flywheel/hood/turret and operator shuffleboard
 * Feeds base limelight to intake vision tracking and driver shuffleboard
 */
public class Limelight extends SubsystemBase {

  //Pulls values from network tables
  //TODO: Add code to pull from both limelights
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry ta = table.getEntry("ta");

  //Creates variables to assign
  private double horizontalOffset;
  private double verticalOffset;
  private double area;

  private boolean targetFound;

  public Limelight() {

  }

  @Override
  public void periodic() {
    // updateReadings();
  }

  public void updateReadings(){
    horizontalOffset = tx.getDouble(0.0);
    verticalOffset = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
  }

  public double getHorizontalOffset(){
    return horizontalOffset;
  }

  public double getVerticalOffset(){
    return verticalOffset;
  }

  public double getArea(){
    return area;
  }

  public boolean isTargetFound(){
    if(area > 0){
      targetFound = true;
    }
    else{
      targetFound = false;
    }
    return targetFound;
  }
}
