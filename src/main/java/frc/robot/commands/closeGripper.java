package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.GripperConstants;
import frc.robot.subsystems.Gripper;

public class closeGripper extends CommandBase {

  private Gripper gripper;
  
  public closeGripper(Gripper gripper) {

    this.gripper = gripper;
    addRequirements(gripper);
  }

  @Override
  public void execute() {
    if(gripper.getDoWeGripACone())
      gripper.putGripperInPose(GripperConstants.coneGrabingSetPoint);
    else
      gripper.putGripperInPose(GripperConstants.cubeGrabingSetPoint);
  }
  
  @Override
  public boolean isFinished() {
    return true;
  }
}
