package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.ShuffleboardConstants;
import frc.robot.commands.chassis.drivetrain.DrivetrainStopCommand;
import frc.robot.commands.indexer.feeder.FeederStopCommand;
import frc.robot.commands.indexer.hopper.HopperStopCommand;
import frc.robot.commands.shooter.flywheel.FlywheelStopCommand;
import frc.robot.commands.shooter.hood.HoodStopCommand;
import frc.robot.commands.shooter.turret.TurretStopCommand;
import frc.robot.subsystems.chassis.DrivetrainSubsystem;
import frc.robot.subsystems.chassis.TransmissionSubsystem;
import frc.robot.subsystems.indexer.FeederSubsystem;
import frc.robot.subsystems.indexer.HopperSubsystem;
import frc.robot.subsystems.intake.ArmSubsytem;
import frc.robot.subsystems.intake.RollerSubsystem;
import frc.robot.subsystems.managers.FlywheelVelocityManager;
import frc.robot.subsystems.managers.HoodPositionManager;
import frc.robot.subsystems.managers.TurretPositionManager;
import frc.robot.subsystems.shooter.FlywheelSubsystem;
import frc.robot.subsystems.shooter.HoodSubsystem;
import frc.robot.subsystems.shooter.TurretSubsystem;
import frc.robot.subsystems.vision.TurretLimelight;

/**
 * Subsystems acts as a container for all subsystems.
 */
public class Subsystems {
  public final class Chassis {
    public final DrivetrainSubsystem drivetrain = DrivetrainSubsystem.create();
    public final TransmissionSubsystem transmission = TransmissionSubsystem.create();
  }

  public final class Climber {
  }

  public final class Indexer {
    public final HopperSubsystem hopper = HopperSubsystem.create();
    public final FeederSubsystem feeder = FeederSubsystem.create();

    public Indexer() {
      var hopperLayout = Shuffleboard.getTab(ShuffleboardConstants.kIndexerTab)
          .getLayout("Hopper State", BuiltInLayouts.kList).withSize(2, 1).withPosition(0, 0)
          .withProperties(Map.of("Label position", "LEFT"));
      var flywheelControlLayout = Shuffleboard.getTab(ShuffleboardConstants.kIndexerTab)
          .getLayout("Hopper Control", BuiltInLayouts.kList).withSize(2, 1).withPosition(0, 1)
          .withProperties(Map.of("Label position", "LEFT"));
      hopper.configureShuffleboard(hopperLayout, flywheelControlLayout);

      var feederStateLayout = Shuffleboard.getTab(ShuffleboardConstants.kIndexerTab)
          .getLayout("Feeder State", BuiltInLayouts.kList).withSize(2, 1).withPosition(2, 0)
          .withProperties(Map.of("Label position", "LEFT"));
      var feederControlLayout = Shuffleboard.getTab(ShuffleboardConstants.kIndexerTab)
          .getLayout("Feeder Control", BuiltInLayouts.kList).withSize(2, 1).withPosition(2, 1)
          .withProperties(Map.of("Label position", "LEFT"));
      feeder.configureShuffleboard(feederStateLayout, feederControlLayout);
    }
  }

  public final class Intake {
    public final ArmSubsytem arm = ArmSubsytem.create();
    public final RollerSubsystem roller = RollerSubsystem.create();

    public Intake() {
      var armStateLayout = Shuffleboard.getTab(ShuffleboardConstants.kIntakeTab)
          .getLayout("Arm State", BuiltInLayouts.kList).withSize(2, 2).withPosition(0, 0)
          .withProperties(Map.of("Label position", "LEFT"));
      var armControlLayout = Shuffleboard.getTab(ShuffleboardConstants.kIntakeTab)
          .getLayout("Arm Control", BuiltInLayouts.kList).withSize(2, 2).withPosition(0, 2)
          .withProperties(Map.of("Label position", "LEFT"));
      arm.configureShuffleboard(armStateLayout, armControlLayout);

      var rollerStateLayout = Shuffleboard.getTab(ShuffleboardConstants.kIntakeTab)
          .getLayout("Roller State", BuiltInLayouts.kList).withSize(2, 2).withPosition(2, 0)
          .withProperties(Map.of("Label position", "LEFT"));
      var rollerControlLayout = Shuffleboard.getTab(ShuffleboardConstants.kIntakeTab)
          .getLayout("Roller Control", BuiltInLayouts.kList).withSize(2, 2).withPosition(2, 2)
          .withProperties(Map.of("Label position", "LEFT"));
      roller.configureShuffleboard(rollerStateLayout, rollerControlLayout);
    }
  }

  public final class Managers {
    public final FlywheelVelocityManager flywheelVelocityManager = new FlywheelVelocityManager();
    public final HoodPositionManager hoodPositionManager = new HoodPositionManager();
    public final TurretPositionManager turretPositionManager = new TurretPositionManager();

    public Managers() {
      var flywheelStateLayout = Shuffleboard.getTab(ShuffleboardConstants.kManagerTab)
          .getLayout("Flywheel Manager State", BuiltInLayouts.kList).withSize(2, 1).withPosition(0, 0)
          .withProperties(Map.of("Label position", "LEFT"));
      var flywheelControlLayout = Shuffleboard.getTab(ShuffleboardConstants.kManagerTab)
          .getLayout("Flywheel Manager Control", BuiltInLayouts.kList).withSize(2, 1).withPosition(0, 1)
          .withProperties(Map.of("Label position", "LEFT"));
      flywheelVelocityManager.configureShuffleboard(flywheelStateLayout, flywheelControlLayout);

      var hoodStateLayout = Shuffleboard.getTab(ShuffleboardConstants.kManagerTab)
          .getLayout("Hood Manager State", BuiltInLayouts.kList).withSize(2, 1).withPosition(2, 0)
          .withProperties(Map.of("Label position", "LEFT"));
      var hoodControlLayout = Shuffleboard.getTab(ShuffleboardConstants.kManagerTab)
          .getLayout("Hood Manager Control", BuiltInLayouts.kList).withSize(2, 1).withPosition(2, 1)
          .withProperties(Map.of("Label position", "LEFT"));
      hoodPositionManager.configureShuffleboard(hoodStateLayout, hoodControlLayout);

      var turretStateLayout = Shuffleboard.getTab(ShuffleboardConstants.kManagerTab)
          .getLayout("Turret Manager State", BuiltInLayouts.kList).withSize(2, 1).withPosition(4, 0)
          .withProperties(Map.of("Label position", "LEFT"));
      var turretControlLayout = Shuffleboard.getTab(ShuffleboardConstants.kManagerTab)
          .getLayout("Turret Manager Control", BuiltInLayouts.kList).withSize(2, 1).withPosition(4, 1)
          .withProperties(Map.of("Label position", "LEFT"));
      turretPositionManager.configureShuffleboard(turretStateLayout, turretControlLayout);
    }
  }

  public final class Shooter {
    public final FlywheelSubsystem flywheel = FlywheelSubsystem.create();
    public final HoodSubsystem hood = HoodSubsystem.create();
    public final TurretSubsystem turret = TurretSubsystem.create();

    public Shooter() {
      var flywheelStateLayout = Shuffleboard.getTab(ShuffleboardConstants.kShooterTab)
          .getLayout("Flywheel State", BuiltInLayouts.kList).withSize(2, 3).withPosition(0, 0)
          .withProperties(Map.of("Label position", "LEFT"));
      var flywheelControlLayout = Shuffleboard.getTab(ShuffleboardConstants.kShooterTab)
          .getLayout("Flywheel Control", BuiltInLayouts.kList).withSize(2, 2).withPosition(0, 3)
          .withProperties(Map.of("Label position", "LEFT"));
      flywheel.configureShuffleboard(flywheelStateLayout, flywheelControlLayout);

      var hoodStateLayout = Shuffleboard.getTab(ShuffleboardConstants.kShooterTab)
          .getLayout("Hood State", BuiltInLayouts.kList).withSize(2, 3).withPosition(2, 0)
          .withProperties(Map.of("Label position", "LEFT"));
      var hoodControlLayout = Shuffleboard.getTab(ShuffleboardConstants.kShooterTab)
          .getLayout("Hood Control", BuiltInLayouts.kList).withSize(2, 2).withPosition(2, 3)
          .withProperties(Map.of("Label position", "LEFT"));
      hood.configureShuffleboard(hoodStateLayout, hoodControlLayout);

      var turretStateLayout = Shuffleboard.getTab(ShuffleboardConstants.kShooterTab)
          .getLayout("Turret State", BuiltInLayouts.kList).withSize(2, 3).withPosition(4, 0)
          .withProperties(Map.of("Label position", "LEFT"));
      var turretControlLayout = Shuffleboard.getTab(ShuffleboardConstants.kShooterTab)
          .getLayout("Turret Control", BuiltInLayouts.kList).withSize(2, 2).withPosition(4, 3)
          .withProperties(Map.of("Label position", "LEFT"));
      turret.configureShuffleboard(turretStateLayout, turretControlLayout);
    }
  }

  public final class Vision {
    public final TurretLimelight turretLimelight = new TurretLimelight();

    public Vision() {
      var turretStateLayout = Shuffleboard.getTab(ShuffleboardConstants.kShooterTab)
          .getLayout("Limelight State", BuiltInLayouts.kList).withSize(2, 3).withPosition(6, 0)
          .withProperties(Map.of("Label position", "LEFT"));
      var turretControlLayout = Shuffleboard.getTab(ShuffleboardConstants.kShooterTab)
          .getLayout("Limelight Control", BuiltInLayouts.kList).withSize(2, 2).withPosition(6, 3)
          .withProperties(Map.of("Label position", "LEFT"));
      turretLimelight.configureShuffleboard(turretStateLayout, turretControlLayout);
    }
  }

  public final Chassis chassis = new Chassis();
  public final Climber climber = new Climber();
  public final Indexer indexer = new Indexer();
  public final Intake intake = new Intake();
  public final Managers managers = new Managers();
  public final Shooter shooter = new Shooter();
  public final Vision vision = new Vision();

  public Subsystems() {
    chassis.drivetrain.setDefaultCommand(new DrivetrainStopCommand(chassis.drivetrain));

    indexer.feeder.setDefaultCommand(new FeederStopCommand(indexer.feeder));
    indexer.hopper.setDefaultCommand(new HopperStopCommand(indexer.hopper));

    shooter.flywheel.setDefaultCommand(new FlywheelStopCommand(shooter.flywheel));
    shooter.hood.setDefaultCommand(new HoodStopCommand(shooter.hood));
    shooter.turret.setDefaultCommand(new TurretStopCommand(shooter.turret));
  }
}
