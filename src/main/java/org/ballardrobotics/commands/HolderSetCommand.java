package org.ballardrobotics.commands;

import org.ballardrobotics.subsystems.Holder;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class HolderSetCommand<T> extends InstantCommand {
  public HolderSetCommand(Holder<T> holder, T value) {
    super(() -> holder.set(value), holder);
  }
}
