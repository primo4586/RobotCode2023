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

  public boolean shouldGripCone,
      lastCollect,
      isHolding;
  
  int clicks = 0;

  public Gripper() {
    gripperMotor = new WPI_TalonFX(8);
    gripperMotor.setInverted(true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  /**
   * @return a command that stops the gripper
   */
  public Command stop() {
    return run(()->setSpeed(0.0));
  }

  public void setSpeed(double speed) {
      gripperMotor.set(speed);
  }

  public double getCurrentRead(){
    return gripperMotor.getStatorCurrent();
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
