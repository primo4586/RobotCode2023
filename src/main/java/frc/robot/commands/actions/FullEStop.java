// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.TelescopicArm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FullEStop extends InstantCommand {
  public FullEStop(LilArm lilArm, BigArm bigArm, TelescopicArm telescopicArm, Swerve swerve) {
    addRequirements(lilArm, bigArm, telescopicArm, swerve);
  }
  
  @Override
  public void initialize() {}
}
