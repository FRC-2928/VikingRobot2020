package frc.robot.subsystems;

import java.util.Map;

import org.ballardrobotics.subsystems.Holder;

import edu.wpi.first.wpilibj.Compressor;
import frc.robot.commands.chassis.drivetrain.DrivetrainStopCommand;
import frc.robot.commands.indexer.feeder.FeederStopCommand;
import frc.robot.commands.indexer.hopper.HopperStopCommand;
import frc.robot.commands.shooter.flywheel.FlywheelStopCommand;
import frc.robot.commands.shooter.hood.HoodStopCommand;
import frc.robot.commands.shooter.turret.TurretStopCommand;
import frc.robot.subsystems.chassis.DrivetrainSubsystem;
import frc.robot.subsystems.chassis.TransmissionSubsystem;
import frc.robot.subsystems.holders.FlywheelVelocityHolder;
import frc.robot.subsystems.holders.HoodPositionHolder;
import frc.robot.subsystems.indexer.FeederSubsystem;
import frc.robot.subsystems.indexer.HopperSubsystem;
import frc.robot.subsystems.intake.ArmSubsytem;
import frc.robot.subsystems.intake.RollerSubsystem;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.vision.TurretLimelight;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Subsystems acts as a container for all subsystems.
 */
public class Subsystems implements Loggable {
  public final class Chassis implements Loggable {
    @Log
    public final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.create();
    @Log
    public final TransmissionSubsystem transmission = TransmissionSubsystem.create();

    @Override
    public Map<String, Object> configureLayoutProperties() {
      return Map.of("Label position", "LEFT");
    }
  }

  public final class Climber implements Loggable {
  }

  public final class Holders implements Loggable {
    private Holder<Double> flywheelVelocity = new FlywheelVelocityHolder();
    private Holder<Double> hoodPosition = new HoodPositionHolder();
  }

  public final class Indexer implements Loggable {
    @Log
    public final HopperSubsystem hopper = HopperSubsystem.create();
    @Log
    public final FeederSubsystem feeder = FeederSubsystem.create();
  }

  public final class Intake implements Loggable {
    @Log
    public final ArmSubsytem arm = ArmSubsytem.create();
    @Log
    public final RollerSubsystem roller = RollerSubsystem.create();
  }

  public final class Shooter implements Loggable {
    @Log
    public final FlywheelSubsystem flywheel = FlywheelSubsystem.create();
    @Log
    public final HoodSubsystem hood = HoodSubsystem.create();
    @Log
    public final TurretSubsystem turret = TurretSubsystem.create();
  }

  public final class Vision implements Loggable {
    @Log
    public final TurretLimelight turretLimelight = new TurretLimelight();
  }

  public final Chassis chassis = new Chassis();
  public final Climber climber = new Climber();
  public final Indexer indexer = new Indexer();
  public final Intake intake = new Intake();
  public final Holders holders = new Holders();
  public final Shooter shooter = new Shooter();
  public final Vision vision = new Vision();
  public final Compressor compressor = new Compressor();

  public Subsystems() {
    chassis.drivetrain.setDefaultCommand(new DrivetrainStopCommand(chassis.drivetrain));

    indexer.feeder.setDefaultCommand(new FeederStopCommand(indexer.feeder));
    indexer.hopper.setDefaultCommand(new HopperStopCommand(indexer.hopper));

    shooter.flywheel.setDefaultCommand(new FlywheelStopCommand(shooter.flywheel));
    shooter.hood.setDefaultCommand(new HoodStopCommand(shooter.hood));
    shooter.turret.setDefaultCommand(new TurretStopCommand(shooter.turret));
  }
}
