package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.shooter.HoodSetPositionCommand;
import frc.robot.commands.shooter.TurretSetPositionCommand;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.utilities.LimeliteUtility.LimeliteData;

public class AutoTargetCommand2 extends ParallelCommandGroup {
  private HoodSubsystem m_hood;
  private TurretSubsystem m_turret;
  private Supplier<LimeliteData> m_limeliteDataSupplier;
  private DoubleSupplier m_gyroAngleSupplier;

  public AutoTargetCommand2(HoodSubsystem hood, TurretSubsystem turret, Supplier<LimeliteData> limeliteDataSupplier, DoubleSupplier gyroAngleSupplier) { 
    var hoodCommand = new HoodSetPositionCommand(hood, this::getTargetHoodAngle);
    var turretCommand = new TurretSetPositionCommand(turret, this::getTargetTurrentAngle);
    addCommands(hoodCommand, turretCommand);

    m_hood = hood;
    m_turret = turret;
    m_limeliteDataSupplier = limeliteDataSupplier;
    m_gyroAngleSupplier = gyroAngleSupplier;
  }

  private double getTargetHoodAngle() {
    return 0.0;
  }

  private double getTargetTurrentAngle() {
    var limeliteData = m_limeliteDataSupplier.get();
    var gyroAngle = m_gyroAngleSupplier.getAsDouble();

    double targetAngle = -gyroAngle;
    if (limeliteData.targetFound) {
      double currentAngle = m_turret.getPosition();
      double offset = limeliteData.horizontalOffsetDegrees;
      targetAngle = currentAngle + offset;
    }
    return targetAngle;
  }
}
