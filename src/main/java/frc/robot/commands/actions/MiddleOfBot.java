// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.BigConstants;
import frc.robot.Constants.LilConstants;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.LilArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MiddleOfBot extends SequentialCommandGroup {
  /** Creates a new IntakeParallel. */
  public MiddleOfBot(LilArm lilArm, BigArm bigArm) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(lilArm.closeLilArmSolenoid(), new MoveArmsParallel(bigArm, BigConstants.intakeSetPoint, lilArm, LilConstants.middleOfRobotSetPoint));
  }
}
