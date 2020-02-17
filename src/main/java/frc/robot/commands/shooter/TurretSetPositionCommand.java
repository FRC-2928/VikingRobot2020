package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import org.ballardrobotics.utilities.RotationUtility;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.subsystems.shooter.TurretSubsystem;

public class TurretSetPositionCommand extends CommandBase {
  private TurretSubsystem m_turret;
  private DoubleSupplier m_positionSupplier;

  public TurretSetPositionCommand(TurretSubsystem turret, double positionDegrees) {
    this(turret, () -> positionDegrees);
  }

  public TurretSetPositionCommand(TurretSubsystem turret, DoubleSupplier positionSupplier) {
    addRequirements(turret);

    m_turret = turret;
    m_positionSupplier = positionSupplier;
  }

  @Override
  public void execute() {
    double inputAngle = m_positionSupplier.getAsDouble();
    double currentAngle = m_turret.getMeasuredPosition();
    double targetAngle = RotationUtility.getTargetAngle(inputAngle, currentAngle, -TurretConstants.kMaxAngle, TurretConstants.kMaxAngle);
    m_turret.setPosition(targetAngle);
  }
}
