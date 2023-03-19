package frc.robot.commands.autoCommands;



import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.actions.GroundAuto;
import frc.robot.commands.actions.GroundTele;
import frc.robot.commands.actions.IntakeParallel;
import frc.robot.commands.actions.IntakeSequential;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.actions.groundReturn;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
public class RedCubeUpAndMidd extends SequentialCommandGroup {
  public RedCubeUpAndMidd(Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm, Boolean shouldPutInUpper, boolean areWeCloseToLoadingStation, boolean gripCone) {
    PutItemInTheMiddle putItemInTheMiddle = new PutItemInTheMiddle(lilArm, bigArm, gripper);
    PutItemInTheUpper putItemInTheUpper = new PutItemInTheUpper(bigArm, lilArm, gripper);

    ConditionalCommand puttingItemInPlace = new ConditionalCommand(putItemInTheUpper, putItemInTheMiddle, () -> shouldPutInUpper);
    
    DriveBackAndGround driveBackAndGround = new DriveBackAndGround(swerve, gripper, bigArm, lilArm);
    addCommands(
      Commands.runOnce(() -> {
        gripper.setShouldGripCone(false);
        gripper.turnOnLed();
      }, gripper),
      puttingItemInPlace,
      Commands.waitSeconds(0.3),
      gripper.openGripper(),
      Commands.waitSeconds(0.3),
      lilArm.closeLilArmSolenoid(),
      driveBackAndGround,
      gripper.closeGripper(),
      Commands.waitSeconds(0.2),
      swerve.followTrajectoryModifiedToRedAlliance(PathPlanner.loadPath("upperCubeReturn", AutoConstants.pathConstraints), false)
      .alongWith(new IntakeParallel(lilArm, bigArm)),
      new PutItemInTheMiddle(lilArm, bigArm, gripper),
      lilArm.openLilArmSolenoid(),
      Commands.waitSeconds(0.4),
      gripper.openGripper(),
      Commands.waitSeconds(0.4),
      new IntakeSequential(lilArm, bigArm).alongWith(Commands.waitSeconds(0.2).andThen(gripper.closeGripper()))
    );
  }
}
