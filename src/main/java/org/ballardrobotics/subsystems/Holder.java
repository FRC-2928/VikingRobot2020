package org.ballardrobotics.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Holder<T> extends SubsystemBase {
  protected T m_value;

  public Holder(T defaultValue) {
    m_value = defaultValue;
  }

  public void set(T value) {
    m_value = value;
  }

  public T get() {
    return m_value;
  }
}
