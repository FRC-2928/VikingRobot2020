package frc.robot.commands.shooter.turret;

import java.util.function.Supplier;

import frc.robot.subsystems.shooter.TurretSubsystem;

public class TurretAutoSetCommand extends TurretSetPositionCommand {
  public class SensorState {
    public boolean visionFound;
    public double visionAngle;
    public boolean estimateFresh;
    public double estimateAngle;
    public double drivetrainHeading;
  }

  public TurretAutoSetCommand(TurretSubsystem turret, Supplier<SensorState> supplier) {
    super(turret, () -> getTargetAngle(turret, supplier));
  }

  private static double getTargetAngle(TurretSubsystem turret, Supplier<SensorState> supplier) {
    // TODO implement
    return 0.0;
  }
}
