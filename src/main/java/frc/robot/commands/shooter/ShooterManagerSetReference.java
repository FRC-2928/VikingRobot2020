package frc.robot.commands.shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterManager;

public class ShooterManagerSetReference extends CommandBase {
  private ShooterManager m_shooterManager;
  private DoubleSupplier m_hoodReference;
  private DoubleSupplier m_flywheelReference;

  public ShooterManagerSetReference(ShooterManager shootermanager, double hoodReference, double flywheelReference) {
    this(shootermanager, ()-> hoodReference, () -> flywheelReference);
}

  public ShooterManagerSetReference(ShooterManager shootermanager, DoubleSupplier hoodReferenceSupplier, DoubleSupplier flywheelReferenceSupplier){
    m_shooterManager = shootermanager;
    addRequirements(m_shooterManager);

    m_hoodReference = hoodReferenceSupplier;
    m_flywheelReference = flywheelReferenceSupplier;    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_shooterManager.setFlywheelReference(m_flywheelReference.getAsDouble());
    m_shooterManager.setHoodReferernce(m_hoodReference.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
