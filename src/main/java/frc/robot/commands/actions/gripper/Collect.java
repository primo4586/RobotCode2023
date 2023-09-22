// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions.gripper;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.subsystems.Gripper;

public class Collect extends CommandBase {
  Gripper gripper;

  public Collect(Gripper gripper) {
    this.gripper = gripper;
    addRequirements(gripper);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(gripper.shouldGripCone){
      gripper.setSpeed(0.5);
    }
    else{
      gripper.setSpeed(-0.5);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new Eject(gripper);
    gripper.lastCollect = gripper.getShouldGripCone();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return gripper.getCurrentRead()>30;
  }
}
