// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Gripper extends SubsystemBase {
  private WPI_TalonFX gripperMotor;

  private boolean shouldGripCone,
      isHolding;
  
  int clicks = 0;

  public Gripper() {
    gripperMotor = new WPI_TalonFX(8);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  /**
   * @return a command that makes the gripper collect, and stops the gripper at
   *         the end of it
   */
  public Command collectCommand() {
    isHolding = false;
    return setSpeed(1).until(() -> gripperMotor.getStatorCurrent() > 30).andThen(holdCommand());
  }

  /**
   * @return a command that makes the gripper eject, and stops the gripper at the
   *         end of it
   */
  public Command ejectCommand() {
    isHolding = false;
    return setSpeed(-1);
  }

  /**
   * @return a command that makes the gripper hold, and stops the gripper at the
   *         end of it
   */
  public Command holdCommand() {
    isHolding = true;
    return setSpeed(0.1);
  }

  /**
   * @return a command that stops the gripper
   */
  public Command stop() {
    return setSpeed(0);
  }

  public Command setSpeed(double speed) {
    return run(() -> {
      gripperMotor.set(shouldGripCone ? -speed : speed);
    });
  }


  public Command changeWhatWeGrip() {
    return new InstantCommand(() -> {
      shouldGripCone = !shouldGripCone;
    });
  }

  public boolean getShouldGripCone() {
    return shouldGripCone;
  }

  public InstantCommand setShouldGripConeCommand(boolean shouldGripCone) {
    return new InstantCommand(() -> {
      this.shouldGripCone = shouldGripCone;
    });
  }

  public void setShouldGripCone(boolean shouldGripCone) {
    this.shouldGripCone = shouldGripCone;
  }

  public boolean isHolding() {
    return isHolding;
  }
}
