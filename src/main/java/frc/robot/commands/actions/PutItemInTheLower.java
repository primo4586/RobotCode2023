// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.TelescopicArm;
import frc.robot.Constants.TelescopicArmConstants;
import frc.robot.commands.actions.gripper.Hold;

public class PutItemInTheLower extends SequentialCommandGroup {
  public PutItemInTheLower(LilArm lilArm, BigArm bigArm, Gripper gripper, TelescopicArm telescopicArm) {
    //cone and cube setPoints
    ParallelCommandGroup coneLowerSetPoint = new ParallelCommandGroup(
        bigArm.TurnBigArmToSetpoint(BigConstants.coneLowerSetPoint),
        lilArm.TurnLilArmToSetpoint(LilConstants.coneLowerSetPoint),
        telescopicArm.putTelesInSetpoint(TelescopicArmConstants.coneLowerSetPoint));

    ParallelCommandGroup cubeLowerSetPoint = new ParallelCommandGroup(
        bigArm.TurnBigArmToSetpoint(BigConstants.cubeLowerSetPoint),
        lilArm.TurnLilArmToSetpoint(LilConstants.cubeLowerSetPoint),
        telescopicArm.putTelesInSetpoint(TelescopicArmConstants.coneLowerSetPoint));

    //check if we put cone or cube
    ConditionalCommand putArmsInLowerSetPoint = new ConditionalCommand(coneLowerSetPoint, cubeLowerSetPoint,gripper::getShouldGripCone);

    Hold hold = new Hold(gripper);

    addCommands(
      hold.raceWith(putArmsInLowerSetPoint)
    );
  }
}
