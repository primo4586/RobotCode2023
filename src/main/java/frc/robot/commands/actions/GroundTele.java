package frc.robot.commands.actions;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
public class GroundTele extends SequentialCommandGroup {

  

  public GroundTele(Gripper gripper, LilArm lilArm, BigArm bigArm) {
    MoveArmsParallel moveArmsToGround = new MoveArmsParallel(bigArm, BigArmConstants.groundSetPoint, lilArm, LilArmConstants.groundSetPoint);

    addCommands(
      lilArm.closeLilArmSolenoid(),
      gripper.closeGripper(),
      moveArmsToGround,
      gripper.openGripper(),
      lilArm.openLilArmSolenoid()
    );
  }
}
