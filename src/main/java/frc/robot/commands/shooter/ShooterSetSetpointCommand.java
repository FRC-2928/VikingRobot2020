package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.types.ShooterSetpoint;

public class ShooterSetSetpointCommand extends ScheduleCommand {
  public ShooterSetSetpointCommand(HoodSubsystem hood, TurretSubsystem turret, FlywheelSubsystem flywheel,
  ShooterSetpoint setpoint) {
    this(hood, turret, flywheel, setpoint, new Trigger(() -> true));
  }

  public ShooterSetSetpointCommand(HoodSubsystem hood, TurretSubsystem turret, FlywheelSubsystem flywheel,
  ShooterSetpoint setpoint, Trigger flywheelTrigger) {
    this(hood, turret, flywheel, setpoint.hoodPositionDegrees, setpoint.turretPositionDegrees, setpoint.flywheelVelocityRPM, flywheelTrigger);
  }

  public ShooterSetSetpointCommand(HoodSubsystem hood, TurretSubsystem turret, FlywheelSubsystem flywheel,
      double hoodPositionDeg, double turretPositionDeg, double flywheelVelocityRPM) {
    this(hood, turret, flywheel, hoodPositionDeg, turretPositionDeg, flywheelVelocityRPM, new Trigger(() -> true));
  }

  public ShooterSetSetpointCommand(HoodSubsystem hood, TurretSubsystem turret, FlywheelSubsystem flywheel,
      double hoodPositionDeg, double turretPositionDeg, double flywheelVelocityRPM, Trigger flywheelTrigger) {
    super(
      new HoodSetPositionCommand(hood, hoodPositionDeg),
      new TurretSetPositionCommand(turret, turretPositionDeg),
      new FlywheelSetVelocityCommand(flywheel, flywheelVelocityRPM, flywheelTrigger)
    );
  }
}
