package frc.robot;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class SystemState {
    private static SystemState m_instance;

    public static SystemState getInstance() {
        if (m_instance == null) {
            m_instance = new SystemState();
        }

        return m_instance;
    }

    private final Map<String, String> m_subsystems = new HashMap<>();

    // Set the subsystem state
    // Example - SystemState.getInstance().setSubsystemState(this, currentState.name());
    public void setSubsystemState(Subsystem subsystem, String state){
        String subsystemName = subsystem.getClass().getName();
        m_subsystems.put(subsystemName, state);
    }

    // Get the current state by class
    public String getSubsystemState(Subsystem subsystem) {
        String state = m_subsystems.get(subsystem.getClass().getName());
        return state;
    }

    // Get the current state by class string
    //  Example - String m_subsystemState = SystemState.getInstance().getSubsystemState("ControlPanelSubsystem");
    public String getSubsystemState(String subsystem) {
        String state = m_subsystems.get(subsystem);
        return state;
    }
}    