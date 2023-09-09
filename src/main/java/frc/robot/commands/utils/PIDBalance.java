// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.utils;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class PIDBalance extends CommandBase {
  private Swerve swerve;
  private PIDController balancePID = new PIDController(0.02, 0.0001, 0.32);
  private Timer timer = new Timer();

  public PIDBalance(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    if (Math.abs(swerve.getRoll()) > 2.5) {
      swerve.drive(
          new Translation2d(balancePID.calculate(swerve.getRoll(), 0), 0),
          0,
          true,
          false);
    } else {
      swerve.wheelsInX();
      timer.start();
    }
  }

  @Override
  public void end(boolean interrupted) {
    swerve.drive(new Translation2d(0,0), 45, true, false);
  }

  @Override
  public boolean isFinished() {
    return timer.hasElapsed(0.3);
  }
}
