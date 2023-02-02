package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.Gripper;

public class openGripper extends CommandBase {

  public Gripper gripper;

  public openGripper(Gripper gripper) {

    this.gripper = gripper;
    addRequirements(gripper);
  }

  @Override
  public void execute() {
    gripper.putGripperInPose(GripperConstants.openGripperSetPoint);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
}
