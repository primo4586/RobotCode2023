// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.objectivetracker;

import java.nio.file.Paths;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.javalin.Javalin;
import io.javalin.http.staticfiles.Location;

public interface NodeSelectorIO {
  public static class NodeSelectorIOInputs{
    public long selectedNode = -1;
    public long coneTipped = -1;
  }

  public static class Objective {
    private final IntegerPublisher nodePublisher;
    private final IntegerSubscriber nodeSubscriber;
    private final IntegerPublisher timePublisher;
    private final BooleanPublisher isAutoPublisher;


    
    public double selectedNode = -1;
    public int nodeRow;
    public NodeLevel nodeLevel;

    public Objective(
        int nodeRow,
        NodeLevel nodeLevel,
        boolean lastIntakeFront) {

      var table = NetworkTableInstance.getDefault().getTable("nodeselector");
      nodePublisher = table.getIntegerTopic("node_robot_to_dashboard").publish();
      nodeSubscriber = table.getIntegerTopic("node_dashboard_to_robot").subscribe(-1);
      table.getBooleanTopic("cone_tipped_dashboard_to_robot").subscribe(false);
      timePublisher = table.getIntegerTopic("match_time").publish();
      isAutoPublisher = table.getBooleanTopic("is_auto").publish();

      this.nodeRow = nodeRow;
      this.nodeLevel = nodeLevel;
      // Start server
      var app = Javalin.create(
          config -> {
            config.staticFiles.add(
                Paths.get(
                    Filesystem.getDeployDirectory().getAbsolutePath().toString(),
                    "nodeselector")
                    .toString(),
                Location.EXTERNAL);
          });
      app.start(5800);
    }

    public Objective() {
      this(0, NodeLevel.HYBRID, false);
    }

    public boolean isConeNode() {
      return nodeLevel != NodeLevel.HYBRID
          && (nodeRow == 0
              || nodeRow == 2
              || nodeRow == 3
              || nodeRow == 5
              || nodeRow == 6
              || nodeRow == 8);
    }

    public int getNodeRow() {
      return (this.nodeRow);
    }

    public NodeLevel getNodeLevel() {
      return (this.nodeLevel);
    }

    public void updateInputs() {
      // Read updates from node selector
      timePublisher.set((long) Math.ceil(Math.max(0.0, DriverStation.getMatchTime())));
      isAutoPublisher.set(DriverStation.isAutonomous());
      for (var value : nodeSubscriber.readQueueValues()) {
        selectedNode = value;
      }

      if (selectedNode != -1) {
        if (DriverStation.getAlliance() == Alliance.Blue) {
          nodeRow = 8 - ((int) selectedNode % 9);
        } else {
          nodeRow = (int) selectedNode % 9;
        }
        if (selectedNode < 9) {
          nodeLevel = NodeLevel.HYBRID;
        } else if (selectedNode < 18) {
          nodeLevel = NodeLevel.MID;
        } else {
          nodeLevel = NodeLevel.HIGH;
        }
        selectedNode = -1;
      }

      // Send current node to selector
      {
        int selected;
        if (DriverStation.getAlliance() == Alliance.Blue) {
          selected = 8 - nodeRow;
        } else {
          selected = nodeRow;
        }
        switch (nodeLevel) {
          case HYBRID -> selected += 0;
          case MID -> selected += 9;
          case HIGH -> selected += 18;
        }
        setSelected(selected);
      }
      putObjectiveAsText();
    }
    
    public void setSelected(long selected) {
      nodePublisher.set(selected);
    }

    public void putObjectiveAsText(){
      // Send current node as text
        String text = "";
        switch (nodeLevel) {
          case HYBRID -> text += "HYBRID";
          case MID -> text += "MID";
          case HIGH -> text += "HIGH";
        }
        text += ", ";
        if (nodeRow < 3) {
          text += DriverStation.getAlliance() == Alliance.Red ? "LEFT" : "RIGHT";
        } else if (nodeRow < 6) {
          text += "CO-OP";
        } else {
          text += DriverStation.getAlliance() == Alliance.Red ? "RIGHT" : "LEFT";
        }
        text += " grid, ";
        if (nodeRow == 1 || nodeRow == 4 || nodeRow == 7) {
          text += nodeLevel == NodeLevel.HYBRID ? "CENTER" : "CUBE";
        } else if (nodeRow == 0 || nodeRow == 3 || nodeRow == 6) {
          text += DriverStation.getAlliance() == Alliance.Red ? "LEFT" : "RIGHT";
          text += nodeLevel == NodeLevel.HYBRID ? "" : " CONE";
        } else {
          text += DriverStation.getAlliance() == Alliance.Red ? "RIGHT" : "LEFT";
          text += nodeLevel == NodeLevel.HYBRID ? "" : " CONE";
        }
        text += " node";
        SmartDashboard.putNumber("test", nodeRow);
        SmartDashboard.putString("Selected Node", text);
    }


  }

  public static enum NodeLevel {
    HYBRID,
    MID,
    HIGH
  }

  public default void updateInputs(NodeSelectorIOInputs inputs) {}

  public default void setSelected(long selected) {}

  public default void setConeOrientation(boolean tipped) {}
}
