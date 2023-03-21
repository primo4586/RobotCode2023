package frc.robot.commands.actions;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.commands.MoveArmsToSetPointsBigFirst;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
public class GroundAuto extends SequentialCommandGroup {
  public GroundAuto(Gripper gripper, LilArm lilArm, BigArm bigArm) {
    IntakeParallel grabItemFromIntake = new IntakeParallel(lilArm, bigArm);
    MoveArmsParallelOnlyForGroundAuto moveArmsToGround = new MoveArmsParallelOnlyForGroundAuto(bigArm, BigArmConstants.groundSetPoint, lilArm, LilArmConstants.groundSetPoint);
    //MoveArmsToSetPointsBigFirst moveArmsToGround = new MoveArmsToSetPointsBigFirst(bigArm, BigArmConstants.groundSetPoint, lilArm, LilArmConstants.groundSetPoint);
    //MoveArmsToSetPointsLilFirst moveArmsToGround2  =new MoveArmsToSetPointsLilFirst(bigArm, BigArmConstants.groundSetPoint2, lilArm, LilArmConstants.groundSetPoint);
    //MoveArmsToSetPointsLilFirst moveArmsToGround3  =new MoveArmsToSetPointsLilFirst(bigArm, BigArmConstants.groundSetPoint2, lilArm, LilArmConstants.groundSetPoint);

    addCommands(
      lilArm.closeLilArmSolenoid(),
      gripper.closeGripper(),
      moveArmsToGround.alongWith(Commands.waitSeconds(0.4).andThen(gripper.openGripper())),
      //grabItemFromIntake,
      //gripper.openGripper(),
      //moveArmsToGround2,
      lilArm.openLilArmSolenoid()
    );
  }
}
