// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import frc.robot.util.MovingAverage;

public class MaybeBetterCharge extends CommandBase {
  private boolean startedClimbing = false;
  private double lastRobotAngle;
  private Swerve swerve;
  private final int direction;
  private MovingAverage rollVelocityAverage = new MovingAverage(10);

  public MaybeBetterCharge(boolean shouldDriveAwayFromDriver, Swerve swerve) {
    this.swerve = swerve;
     lastRobotAngle = swerve.getRoll();

     if(shouldDriveAwayFromDriver)
     direction = -1;
     else
     direction = 1;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
  //   rollVelocityAverage.addNumber((swerve.getRoll() - lastRobotAngle) / 0.02);
    swerve.drive(new Translation2d(1*direction,0), 0, true, false);

    // SmartDashboard.putBoolean("startedClimbing", startedClimbing);
    // SmartDashboard.putNumber("rollAvg", rollVelocityAverage.getAverage());
    // if(Math.abs(swerve.getRoll()) > 9){
    //   startedClimbing = true;
    // }

    // lastRobotAngle = swerve.getRoll();
  }
  
  @Override
  public void end(boolean interrupted) {
    //swerve.drive(new Translation2d(0,0), 0, true, true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(rollVelocityAverage.getAverage() > 8 && startedClimbing){
    //   return true;
    // }
    return false;
  }
}
