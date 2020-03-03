package org.ballardrobotics.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * SuppliedCommand are commands which required information continuously
 * supplied to them in order to function.
 * 
 * @param <T>
 */
public class SuppliedCommand<T> extends CommandBase {
    protected Supplier<T> m_supplier;

    public SuppliedCommand(Supplier<T> supplier) {
        setSupplier(supplier);;
    }

    public void setSupplier(Supplier<T> supplier) {
        m_supplier = supplier;
    }
}
