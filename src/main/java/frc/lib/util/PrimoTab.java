package frc.lib.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PrimoTab {
    private String title;
    private Map<String, GenericEntry> entryMap;
    private ShuffleboardTab tab;
    public PrimoTab(String title){
        this.title = title;
        entryMap = new HashMap<>();
        tab = Shuffleboard.getTab(title);
    }

    public ShuffleboardTab getTab() {
        return tab;
    }

    public GenericEntry addEntry(String name) {
        if (entryMap.containsKey(name)) {
            GenericEntry entry = entryMap.get(name);
            return entry;
        }
        GenericEntry entry = tab.add(name, 0).getEntry();
        entryMap.put(name, entry);
        return entry;
    }
    public String getTitle() {
        return title;
    }
    
}
