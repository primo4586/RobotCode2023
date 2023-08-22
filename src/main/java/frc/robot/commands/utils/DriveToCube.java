// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class DriveToCube extends CommandBase {
  /** Creates a new DriveToCube. */
  Swerve swerve;

  public DriveToCube(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.drive(new Translation2d(-swerve.getX() + -SwerveConstants.minAutoCollectSpeed, 0),
        swerve.getY() + swerve.getY() < 0 ? SwerveConstants.minAutoCollectRotation: -SwerveConstants.minAutoCollectRotation,
        false, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.stopModules();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return swerve.getTargetExist() && swerve.getTargetClass() == "cube";
  }
}
