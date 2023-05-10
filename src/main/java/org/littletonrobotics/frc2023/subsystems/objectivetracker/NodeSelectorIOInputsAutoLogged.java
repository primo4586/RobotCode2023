package org.littletonrobotics.frc2023.subsystems.objectivetracker;

import java.lang.Cloneable;
import java.lang.Override;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public class NodeSelectorIOInputsAutoLogged extends NodeSelectorIO.NodeSelectorIOInputs implements LoggableInputs, Cloneable {
  @Override
  public void toLog(LogTable table) {
    table.put("SelectedNode", selectedNode);
    table.put("ConeTipped", coneTipped);
  }

  @Override
  public void fromLog(LogTable table) {
    selectedNode = table.getInteger("SelectedNode", selectedNode);
    coneTipped = table.getInteger("ConeTipped", coneTipped);
  }

  public NodeSelectorIOInputsAutoLogged clone() {
    NodeSelectorIOInputsAutoLogged copy = new NodeSelectorIOInputsAutoLogged();
    copy.selectedNode = this.selectedNode;
    copy.coneTipped = this.coneTipped;
    return copy;
  }
}
