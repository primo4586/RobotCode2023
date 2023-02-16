package frc.robot.commands.autoCommands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.commands.MoveArmsToSetPoints;
import frc.robot.commands.actions.PutItemInTheMiddle;
import frc.robot.commands.actions.PutItemInTheUpper;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

public class GamePieceThenCharge extends SequentialCommandGroup {
  public GamePieceThenCharge(Gripper gripper, BigArm bigArm, LilArm lilArm, Swerve swerve, BooleanSupplier shouldPutInUpper) {

    PutItemInTheMiddle putItemInTheMiddle = new PutItemInTheMiddle(lilArm, bigArm, gripper);
    PutItemInTheUpper putItemInTheUpper = new PutItemInTheUpper(bigArm, lilArm, gripper);

    ConditionalCommand puttingItemInPlace = new ConditionalCommand(putItemInTheUpper, putItemInTheMiddle, shouldPutInUpper);

    MoveArmsToSetPoints moveArmsToMiddleOfBot = new MoveArmsToSetPoints(bigArm, BigArmConstants.middleOfRobotSetPoint, lilArm, LilArmConstants.middleOfRobotSetPoint);

    addCommands(
      puttingItemInPlace,
      moveArmsToMiddleOfBot,
      //add driving to charge station
      swerve.chargeStationAlign()
    );
  }
}
