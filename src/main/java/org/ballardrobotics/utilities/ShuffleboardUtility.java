package org.ballardrobotics.utilities;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

public class ShuffleboardUtility {
    private static Map<String, Map<String, NetworkTableEntry>> map;

    public static NetworkTableEntry getEntry(String tab, String name, Object defaultValue) {
        if (map == null) {
            map = new HashMap<>();
        }

        if (!map.containsKey(tab)) {
            map.put(tab, new HashMap<>());
        }

        var innerMap = map.get(tab);
        if (!innerMap.containsKey(name)) {
            var entry = Shuffleboard.getTab(tab).add(name, defaultValue).getEntry();
            innerMap.put(name, entry);
        }
        return innerMap.get(name);
    }

    public static void putData(String tab, String name, Sendable value) {
        var dataTable = NetworkTableInstance.getDefault().getTable("Shuffleboard").getSubTable(tab).getSubTable(name);
        SendableRegistry.publish(value, dataTable);
        dataTable.getEntry(".name").setString(name);
    }

    public static void putNumber(String tab, String name, Number value) {
        getEntry(tab, name, 0.0).forceSetNumber(value);
    }

    public static void putString(String tab, String name, String value) {
        getEntry(tab, name, "").forceSetString(value);
    }

    public static void putBoolean(String tab, String name, boolean value) {
        getEntry(tab, name, false).forceSetBoolean(value);
    }
}
