package frc.robot.commands.auto;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.utilities.LimeliteUtility.LimeliteData;

public class AutoTargetCommand extends CommandBase {
  private HoodSubsystem m_hood;
  private TurretSubsystem m_turret;
  private Supplier<LimeliteData> m_limeliteDataSupplier;
  private DoubleSupplier m_gyroAngleSupplier;

  public AutoTargetCommand(HoodSubsystem hood, TurretSubsystem turret, Supplier<LimeliteData> limeliteDataSupplier, DoubleSupplier gyroAngleSupplier) {
    addRequirements(hood, turret);

    m_hood = hood;
    m_turret = turret;
    m_limeliteDataSupplier = limeliteDataSupplier;
    m_gyroAngleSupplier = gyroAngleSupplier;
  }

  @Override
  public void execute() {
    m_hood.setPosition(getTargetHoodAngle());
    m_turret.setPosition(getTargetTurrentAngle());
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
