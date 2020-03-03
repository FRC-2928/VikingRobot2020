package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.intake.ArmSetStateCommand;

public class ArmSubsytem extends SubsystemBase {
  private Solenoid m_backPiston;
  private Solenoid m_frontPiston;

  public enum State {
    Stowed,
    Pickup,
  }
  private State m_state = State.Stowed;

  public static ArmSubsytem create() {
    var backPiston = new Solenoid(ArmConstants.kBackSolenoidPort);
    var frontPiston = new Solenoid(ArmConstants.kFrontSolenoidPort);
    return new ArmSubsytem(backPiston, frontPiston);
  }

  public ArmSubsytem(Solenoid backPiston, Solenoid frontPiston) {
    m_backPiston = backPiston;
    m_frontPiston = frontPiston;
  }

  public void configureShuffleboard(ShuffleboardLayout stateLayout, ShuffleboardLayout controlLayout) {
    stateLayout.addBoolean("back_piston", m_backPiston::get);
    stateLayout.addBoolean("front_piston", m_frontPiston::get);
    stateLayout.addString("state", () -> m_state.toString());

    controlLayout.add("stowed", new ArmSetStateCommand(this, State.Stowed));
    controlLayout.add("pickup", new ArmSetStateCommand(this, State.Pickup));
  }

  public void setState(State state) {
    switch(state) {
      case Stowed:
        m_backPiston.set(false);
        m_frontPiston.set(false);
        break;
      case Pickup:
        m_backPiston.set(true);
        m_frontPiston.set(true);
        break;
    }
    m_state = state;
  }

  public State getState() {
    return m_state;
  }
}
