// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.Constants.GripperConstants;

public class Gripper extends PIDSubsystem {
  private WPI_TalonSRX  gripperMotor;
  private Encoder gripperEncoder;
  private SimpleMotorFeedforward gripperMotorFeedforward;

  /** Creates a new gripper. */
  public Gripper() {
    super(GripperConstants.gripperPID); // TODO: change constants

        gripperMotor = new WPI_TalonSRX(GripperConstants.gripperMotorPort);
        gripperMotorFeedforward = new SimpleMotorFeedforward(GripperConstants.gripperKs, GripperConstants.gripperKv);

        gripperEncoder = new Encoder(GripperConstants.gripperIncoderPortA, GripperConstants.gripperIncoderPortB);
        gripperEncoder.setDistancePerPulse(0.0); // TODO: add distancePerPulse
  }

  @Override
  public void useOutput(double output, double setpoint) {
    // Use the output here
    gripperMotor.setVoltage(output + gripperMotorFeedforward.calculate(setpoint));

  }

  @Override
  public double getMeasurement() {
    // Return the process variable measurement here
    return gripperEncoder.getRate();
  }
}
