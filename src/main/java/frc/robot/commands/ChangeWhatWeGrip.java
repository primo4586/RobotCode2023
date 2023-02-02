package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class ChangeWhatWeGrip extends CommandBase {
  private Gripper gripper;

  public ChangeWhatWeGrip(Gripper gripper) {

    this.gripper = gripper;
    addRequirements(gripper);
  }

  @Override
  public void execute() {
    gripper.changeDoWeGripACone();
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
