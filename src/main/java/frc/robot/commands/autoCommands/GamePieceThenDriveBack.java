package frc.robot.commands.autoCommands;


import com.pathplanner.lib.PathPlanner;
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
  public GamePieceThenDriveBack(Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm, Boolean shouldPutInUpper, boolean areWeCloseToLoadingStation) {

    PutItemInTheMiddle putItemInTheMiddle = new PutItemInTheMiddle(lilArm, bigArm, gripper);
    PutItemInTheUpper putItemInTheUpper = new PutItemInTheUpper(bigArm, lilArm, gripper);

    ConditionalCommand puttingItemInPlace = new ConditionalCommand(putItemInTheUpper, putItemInTheMiddle, () -> shouldPutInUpper);

    MoveArmsToTheGround moveArmsToTheGround = new MoveArmsToTheGround(gripper, lilArm, bigArm);

    PathPlannerTrajectory trajectory;

    if(areWeCloseToLoadingStation){
      if(gripper.getShouldGripCone())
        trajectory = PathPlanner.loadPath("upperCone", null);
      else
        trajectory = PathPlanner.loadPath("upperCube", null);
    }
    else{
      if(gripper.getShouldGripCone())
        trajectory = PathPlanner.loadPath("lowerCone", null);
      else
        trajectory = PathPlanner.loadPath("lowerCube", null);
    }

    addCommands(
      puttingItemInPlace,
      swerve.followTrajectoryModifiedToAlliance(trajectory, true),
      moveArmsToTheGround
    );
  }
}