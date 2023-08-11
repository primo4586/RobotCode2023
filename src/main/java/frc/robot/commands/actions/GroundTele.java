package frc.robot.commands.actions;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
public class GroundTele extends SequentialCommandGroup {

  

  public GroundTele(Gripper gripper, LilArm lilArm, BigArm bigArm) {
    MoveArmsParallel moveArmsToGround = new MoveArmsParallel(bigArm, BigConstants.groundSetPoint, lilArm, LilConstants.groundSetPoint);

    addCommands(
      lilArm.closeLilArmSolenoid(),
      moveArmsToGround,
      gripper.getCollectCommand(),
      lilArm.openLilArmSolenoid()
    );
  }
}
