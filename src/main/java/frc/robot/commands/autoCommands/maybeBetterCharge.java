// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autoCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Swerve;
import frc.robot.util.MovingAverage;

public class maybeBetterCharge extends CommandBase {
  private boolean shouldDriveBack;
  private boolean startedClimbing = false;
  private double lastRobotAngle;
  private final Swerve swerve;
  private final int direction;
  private MovingAverage rollVelocityAverage = new MovingAverage(30);

  public maybeBetterCharge(boolean shouldDriveBack, Swerve swerve) {
    this.swerve = swerve;
    this.shouldDriveBack = shouldDriveBack;
    lastRobotAngle = swerve.getRoll();

    if(shouldDriveBack)
    direction = -1;
    else
    direction = 1;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    rollVelocityAverage.addNumber((swerve.getRoll() - lastRobotAngle) / 0.02);
    swerve.drive(new Translation2d(1*direction,0), 0, true, false);

    if(Math.abs(swerve.getRoll()) > 9){
      startedClimbing = true;
    }

    lastRobotAngle = swerve.getRoll();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new PIDBalance(swerve);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(rollVelocityAverage.getAverage() > 8 && startedClimbing){
      swerve.drive(new Translation2d(0,0), 45, true, false);
      return true;
    }
    return false;
  }
}
