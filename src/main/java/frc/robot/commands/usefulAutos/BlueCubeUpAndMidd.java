package frc.robot.commands.usefulAutos;

import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.actions.IntakeParallel;
import frc.robot.commands.actions.IntakeSequential;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.usefulAutos.utils.BlueUpperDriveBackAndGround;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

public class BlueCubeUpAndMidd extends SequentialCommandGroup {
  public BlueCubeUpAndMidd(Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm) {

    PutItemInTheMiddle putItemInTheMiddle = new PutItemInTheMiddle(lilArm, bigArm, gripper);
    PutItemInTheUpper putItemInTheUpper = new PutItemInTheUpper(bigArm, lilArm, gripper);

    BlueUpperDriveBackAndGround driveBackAndGround = new BlueUpperDriveBackAndGround(swerve, gripper, bigArm, lilArm, true);
    addCommands(
        Commands.runOnce(() -> {
          gripper.setShouldGripCone(false);
          gripper.turnOnLed();
        }, gripper),
        
        putItemInTheUpper,
        Commands.waitSeconds(0.3),
        gripper.openGripper(),
        Commands.waitSeconds(0.3),
        lilArm.closeLilArmSolenoid(),
        driveBackAndGround,
        gripper.closeGripper(),
        Commands.waitSeconds(0.2),
        swerve.followTrajectory(PathPlanner.loadPath("blueUpperCubeReturn", AutoConstants.pathConstraints), false)
            .alongWith(new IntakeParallel(lilArm, bigArm)),
        putItemInTheMiddle,
        lilArm.openLilArmSolenoid(),
        Commands.waitSeconds(0.4),
        gripper.openGripper(),
        Commands.waitSeconds(0.4),
        new IntakeSequential(lilArm, bigArm).alongWith(Commands.waitSeconds(0.2).andThen(gripper.closeGripper())));
  }
}
