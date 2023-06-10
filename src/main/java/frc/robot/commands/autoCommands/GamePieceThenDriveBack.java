package frc.robot.commands.autoCommands;



import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.actions.GroundAuto;
import frc.robot.commands.actions.IntakeSequential;
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

    IntakeSequential moveToIntakeNoOpen = new IntakeSequential(lilArm, bigArm);

    GroundAuto groundAuto = new GroundAuto(gripper, lilArm, bigArm, true);

    addCommands(
      Commands.runOnce(() -> {
        gripper.setShouldGripCone(gripCone);
        gripper.turnOnLed();
      }, gripper),
      puttingItemInPlace,
      Commands.waitSeconds(1),
      gripper.openGripper(),
      Commands.waitSeconds(0.5),
      lilArm.closeLilArmSolenoid(),
      //swerve.driveForTimeAtSpeed(new Translation2d(-1.25, 0), 2.7),
      new ParallelCommandGroup(swerve.driveForTimeAtSpeed(new Translation2d(-2.0, 0), 2.5),moveToIntakeNoOpen, Commands.waitSeconds(0.5).andThen(gripper.closeGripper())),
      groundAuto
      //gripper.closeGripper(),
      //new groundReturn(lilArm, bigArm)
      //new ParallelCommandGroup(swerve.driveForTimeAtSpeed(new Translation2d(-1.25, 0), 2.7),new GroundAuto(gripper, lilArm, bigArm))
      //swerve.driveForTimeAtSpeed(new Translation2d(-1.25,0), 1.5)
      //new ParallelCommandGroup(swerve.driveForTimeAtSpeed(new Translation2d(-1.25, 0), 2.7),new GroundTele(gripper, lilArm, bigArm), Commands.waitSeconds(0.5)),
      //groundAuto
      );
  }
}
