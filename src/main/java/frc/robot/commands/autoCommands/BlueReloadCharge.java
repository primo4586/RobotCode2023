package frc.robot.commands.autoCommands;
import com.pathplanner.lib.PathPlanner;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.actions.IntakeParallel;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.commands.usefulAutos.utils.BlueUpperDriveBackAndGround;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
public class BlueReloadCharge extends SequentialCommandGroup {
  public BlueReloadCharge(Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm, Boolean shouldPutInUpper, boolean areWeCloseToLoadingStation, boolean gripCone) {

    PutItemInTheMiddle putItemInTheMiddle = new PutItemInTheMiddle(lilArm, bigArm, gripper);
    PutItemInTheUpper putItemInTheUpper = new PutItemInTheUpper(bigArm, lilArm, gripper);

    ConditionalCommand puttingItemInPlace = new ConditionalCommand(putItemInTheUpper, putItemInTheMiddle, () -> shouldPutInUpper);
    
    BlueUpperDriveBackAndGround driveBackAndGround = new BlueUpperDriveBackAndGround(swerve, gripper, bigArm, lilArm, true);

    ChargeAlignOtherSide chargeAlignOtherSide = new ChargeAlignOtherSide(swerve);

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
      swerve.followTrajectory(PathPlanner.loadPath("cubeCharge", AutoConstants.pathConstraints), false)
      .alongWith(new IntakeParallel(lilArm, bigArm)),
      chargeAlignOtherSide

    );
  }
}
