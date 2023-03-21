package frc.lib.util;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

public class PrimoShuffleboard {

    private Map<String, PrimoTab> tabs;
    private static PrimoShuffleboard instance;

    private PrimoShuffleboard() {
        tabs = new HashMap<>();
    }

    public static PrimoShuffleboard getInstance() {
        if (instance == null)
            instance = new PrimoShuffleboard();
        return instance;
    }

    // Adds or gets a new PrimoTab to avoid adding a tab that already exists and
    // crashing the robot
    public PrimoTab getPrimoTab(String tabName) {
        if (tabs.containsKey(tabName))
            return tabs.get(tabName);

        PrimoTab tab = new PrimoTab(tabName);
        tabs.put(tabName, tab);
        return tab;
    }

    // Switches the dashboard to view a different tab, if it exists
    public void selectTab(String tabName) {
        if (tabs.containsKey(tabName))
            Shuffleboard.selectTab(tabName);
    }

    public PrimoTab getCompetitonBoard() {
        return PrimoShuffleboard.getInstance().getPrimoTab("Competition Dashboard");
    }

    public void initDashboard(Swerve swerve, LilArm lilArm, BigArm bigArm, Gripper gripper, VideoSource camera) {
        PrimoTab tab = getCompetitonBoard();

        try {
            tab.getTab().add("POV: You are Itzik", camera).withPosition(2, 0).withSize(7, 4);
            tab.getTab().addBoolean("Cone?", () -> gripper.getShouldGripCone()).withPosition(9, 0).withSize(1, 4);
            tab.getTab().addDouble("Time left", () -> Timer.getMatchTime()).withPosition(0, 0).withSize(2, 1);

        } catch (IllegalArgumentException alreadyExists) {} // We ignore.
    }

    public String getCompTabTitle() {
        return "Competition Dashboard";
    }

}
