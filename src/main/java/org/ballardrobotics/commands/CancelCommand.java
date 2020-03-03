package org.ballardrobotics.commands;

import java.util.Set;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class CancelCommand extends CommandBase {
    private final Set<Command> m_toCancel;

    public CancelCommand(Command... toCancel) {
        m_toCancel = Set.of(toCancel);
    }
  
    @Override
    public void initialize() {
      for (Command command : m_toCancel) {
        command.cancel();
      }
    }
  
    @Override
    public boolean isFinished() {
      return true;
    }
  
    @Override
    public boolean runsWhenDisabled() {
      return true;
    }
}
