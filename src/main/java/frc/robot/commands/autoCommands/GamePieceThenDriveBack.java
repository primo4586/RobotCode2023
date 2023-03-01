package frc.robot.commands.autoCommands;


import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.actions.GrabItemFromIntakeNoOpen;
import frc.robot.commands.actions.MoveArmsToTheGround;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

public class GamePieceThenDriveBack extends SequentialCommandGroup {
  public GamePieceThenDriveBack(Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm, Boolean shouldPutInUpper, boolean areWeCloseToLoadingStation, boolean gripCone) {

    PutItemInTheMiddle putItemInTheMiddle = new PutItemInTheMiddle(lilArm, bigArm, gripper);
    PutItemInTheUpper putItemInTheUpper = new PutItemInTheUpper(bigArm, lilArm, gripper);

    ConditionalCommand puttingItemInPlace = new ConditionalCommand(putItemInTheUpper, putItemInTheMiddle, () -> shouldPutInUpper);

    MoveArmsToTheGround moveArmsToTheGround = new MoveArmsToTheGround(gripper, lilArm, bigArm);

    GrabItemFromIntakeNoOpen moveToIntakeNoOpen = new GrabItemFromIntakeNoOpen(lilArm, bigArm, gripper);

    PathPlannerTrajectory trajectory;

    // if(areWeCloseToLoadingStation){
    //   if(gripCone)
    //     trajectory = PathPlanner.loadPath("upperCone", AutoConstants.pathConstraints);
    //   else
    //     trajectory = PathPlanner.loadPath("upperCube", AutoConstants.pathConstraints);
    // }
    // else{
    //   if(gripCone)
    //     trajectory = PathPlanner.loadPath("lowerCone", AutoConstants.pathConstraints);
    //   else
    //     trajectory = PathPlanner.loadPath("lowerCube", AutoConstants.pathConstraints);
    // }

    addCommands(
      Commands.runOnce(() -> {
        gripper.setShouldGripCone(gripCone);
        gripper.turnOnLed();
      }, gripper),
      puttingItemInPlace,
      gripper.openGripper(),
      Commands.waitSeconds(0.5),
      lilArm.closeLilArmSolenoid(),
      gripper.closeGripper(),
      swerve.driveForTimeAtSpeed(new Translation2d(-1.25, 0), 2.5).alongWith(moveToIntakeNoOpen)
      // swerve.followTrajectoryModifiedToAlliance(trajectory, true, true).alongWith(moveToIntakeNoOpen),
      // moveArmsToTheGround
    );
  }
}