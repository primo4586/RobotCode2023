// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigArmConstants;
import frc.robot.Constants.LilArmConstants;
import frc.robot.commands.MoveArmsToSetPointsBigFirst;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class groundReturn extends SequentialCommandGroup {
  /** Creates a new groundReturn. */
  public groundReturn(LilArm lilArm, BigArm bigArm,Gripper gripper) {
    
  MoveArmsToSetPointsBigFirst moveArmsToIntake = new MoveArmsToSetPointsBigFirst(bigArm, BigArmConstants.intakeSetPoint, lilArm, LilArmConstants.intakeSetPoint);

  MoveArmsToSetPointsBigFirst returnPoint = new MoveArmsToSetPointsBigFirst(bigArm, BigArmConstants.groundSetPoint, lilArm, LilArmConstants.intakeSetPoint);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
    
    lilArm.closeLilArmSolenoid(),
    //returnPoint,
    moveArmsToIntake
    );
  }
}
