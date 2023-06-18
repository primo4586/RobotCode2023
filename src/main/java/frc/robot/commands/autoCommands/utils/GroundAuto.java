package frc.robot.commands.autoCommands.utils;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.commands.actions.MoveArmsParallelOnlyForGroundAuto;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
public class GroundAuto extends SequentialCommandGroup {

  //TODO: fix setPoints

  public GroundAuto(Gripper gripper, LilArm lilArm, BigArm bigArm, boolean shouldExtendLil) {
    MoveArmsParallelOnlyForGroundAuto moveArmsToGround = new MoveArmsParallelOnlyForGroundAuto(bigArm, BigArmConstants.groundSetPoint, lilArm, LilArmConstants.groundSetPoint);

    ConditionalCommand openLilArm = new ConditionalCommand(lilArm.openLilArmSolenoid(), Commands.none(), ()-> shouldExtendLil);

    addCommands(
      lilArm.closeLilArmSolenoid(),
      gripper.closeGripper(),
      moveArmsToGround.alongWith(Commands.waitSeconds(0.4).andThen(gripper.openGripper())),
      openLilArm
    );
  }
}
