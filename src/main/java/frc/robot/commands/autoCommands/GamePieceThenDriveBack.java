package frc.robot.commands.autoCommands;


import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.MoveArmsToTheGround;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

public class GamePieceThenDriveBack extends SequentialCommandGroup {
  public GamePieceThenDriveBack(Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm, Boolean shouldPutInUpper, PathPlannerTrajectory trajectory) {

    PutItemInTheMiddle putItemInTheMiddle = new PutItemInTheMiddle(lilArm, bigArm, gripper);
    PutItemInTheUpper putItemInTheUpper = new PutItemInTheUpper(bigArm, lilArm, gripper);

    ConditionalCommand puttingItemInPlace = new ConditionalCommand(putItemInTheUpper, putItemInTheMiddle, () -> shouldPutInUpper);

    MoveArmsToTheGround moveArmsToTheGround = new MoveArmsToTheGround(gripper, lilArm, bigArm);

    addCommands(
      puttingItemInPlace,
      swerve.followTrajectoryModifiedToAlliance(trajectory, true),
      moveArmsToTheGround
    );
  }
}