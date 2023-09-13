// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants.BigConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.Constants.TelescopicArmConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.TelescopicArm;

public class MiddleOfBot extends ParallelCommandGroup {
  /** Creates a new MiddleOfBot. */
  public MiddleOfBot(LilArm lilArm, BigArm bigArm, TelescopicArm telescopicArm, Gripper gripper) {
    addCommands(
      gripper.holdCommand(),
      telescopicArm.putTelesInSetpoint(TelescopicArmConstants.middleOfRobotSetPoint),
      bigArm.TurnBigArmToSetpoint(BigConstants.intakeSetPoint),
      lilArm.TurnLilArmToSetpoint(LilConstants.middleOfRobotSetPoint)
    );
  }
}
