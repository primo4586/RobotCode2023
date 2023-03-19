package frc.robot.commands.actions;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.commands.MoveArmsToSetPointsBigFirst;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
public class GroundTele extends SequentialCommandGroup {
  public GroundTele(Gripper gripper, LilArm lilArm, BigArm bigArm) {

    IntakeParallel grabItemFromIntake = new IntakeParallel(lilArm, bigArm);
    MoveArmsToSetPointsBigFirst moveArmsToGround = new MoveArmsToSetPointsBigFirst(bigArm, BigArmConstants.groundSetPoint, lilArm, LilArmConstants.groundSetPoint);
    //MoveArmsToSetPointsLilFirst moveArmsToGround2  =new MoveArmsToSetPointsLilFirst(bigArm, BigArmConstants.groundSetPoint2, lilArm, LilArmConstants.groundSetPoint);
    //MoveArmsToSetPointsLilFirst moveArmsToGround3  =new MoveArmsToSetPointsLilFirst(bigArm, BigArmConstants.groundSetPoint2, lilArm, LilArmConstants.groundSetPoint);

    addCommands(
      lilArm.closeLilArmSolenoid(),
      gripper.closeGripper(),
      //grabItemFromIntake,
      moveArmsToGround,
      //gripper.openGripper(),
      //moveArmsToGround2,
      lilArm.openLilArmSolenoid()
    );
  }
}
