// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.Swerve;
import frc.robot.vision.LimeLight;

public class DriveToCube extends CommandBase {
  /** Creates a new DriveToCube. */
  Swerve swerve;
  Gripper gripper;
  LimeLight limeLight;

  public DriveToCube(Swerve swerve, Gripper gripper, LimeLight limeLight) {
    this.gripper = gripper;
    this.swerve = swerve;
    this.limeLight = limeLight;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    limeLight.cubeLimeLight();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limeLight.cubeExist()) {
      swerve.drive(new Translation2d(-limeLight.getCubeX() + -SwerveConstants.minAutoCollectSpeed, 0),
          limeLight.getCubeY() + limeLight.getCubeY() < 0 ? SwerveConstants.minAutoCollectRotation
              : -SwerveConstants.minAutoCollectRotation,
          false, false);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    limeLight.camLimeLight();
    swerve.drive(new Translation2d(0,0), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gripper.isHolding() || !limeLight.getTargetExist(); 
  }
}