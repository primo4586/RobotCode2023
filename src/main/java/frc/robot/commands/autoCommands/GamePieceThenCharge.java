package frc.robot.commands.autoCommands;


import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.commands.MoveArmsToSetPointsBigFirst;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

public class GamePieceThenCharge extends SequentialCommandGroup {
  public GamePieceThenCharge(Gripper gripper, BigArm bigArm, LilArm lilArm, Swerve swerve, Boolean shouldPutInUpper) {

    PutItemInTheMiddle putItemInTheMiddle = new PutItemInTheMiddle(lilArm, bigArm, gripper);
    PutItemInTheUpper putItemInTheUpper = new PutItemInTheUpper(bigArm, lilArm, gripper);

    ConditionalCommand puttingItemInPlace = new ConditionalCommand(putItemInTheUpper, putItemInTheMiddle,()-> shouldPutInUpper);

    MoveArmsToSetPointsBigFirst moveArmsToMiddleOfBot = new MoveArmsToSetPointsBigFirst(bigArm, BigArmConstants.intakeSetPoint, lilArm, LilArmConstants.intakeSetPoint);

    
    addCommands(
      puttingItemInPlace,
      moveArmsToMiddleOfBot,
      new ChargeAlignOtherSide(swerve)
    );
  }
}
