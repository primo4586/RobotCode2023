// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;

public class DaulArmSim extends SubsystemBase {

  BigArm bigArm;
  LilArm lilArm;

  private final SingleJointedArmSim bigArmSim = new SingleJointedArmSim(
    DCMotor.getNEO(1),
    80*3,
    SingleJointedArmSim.estimateMOI(0.85, 6.2),
    0.85,
    Units.degreesToRadians(-25555),
    Units.degreesToRadians(222225),
    false);

private final Mechanism2d bigMech2d = new Mechanism2d(60, 60);
private final MechanismRoot2d bigArmPivot = bigMech2d.getRoot("ArmPivot", 30, 30);
private final MechanismLigament2d bigArmTower = bigArmPivot.append(new MechanismLigament2d("ArmTower", 30, -90));
private final MechanismLigament2d bigArmLigament2d = bigArmPivot.append(
    new MechanismLigament2d(
        "Arm",
        30,
        Units.radiansToDegrees(bigArmSim.getAngleRads()),
        6,
        new Color8Bit(Color.kYellow)));

  private final SingleJointedArmSim lilArmSim = new SingleJointedArmSim(
      DCMotor.getNEO(1),
    5*30,
      SingleJointedArmSim.estimateMOI(0.700, 3.1),
      0.7,
      Units.degreesToRadians(-25555),
      Units.degreesToRadians(222225),
      false);

  private final Mechanism2d lilMech2d = new Mechanism2d(60, 60);
  private final MechanismLigament2d lilArmlLigament2d = bigArmLigament2d.append(
      new MechanismLigament2d(
          "Arm",
          30,
          Units.radiansToDegrees(lilArmSim.getAngleRads()),
          6,
          new Color8Bit(Color.kYellow)));

  public DaulArmSim(BigArm bigArm, LilArm lilArm) {
    this.bigArm = bigArm;
    this.lilArm = lilArm;
    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Big Arm Sim", bigMech2d);
    bigArmTower.setColor(new Color8Bit(Color.kBlue));

    // Put Mechanism 2d to SmartDashboard
    SmartDashboard.putData("Big Arm Sim", lilMech2d);
  }

  @Override
  public void simulationPeriodic() {
    bigArmSim.setInput(bigArm.getSpeed() * RobotController.getBatteryVoltage());
    bigArmSim.update(0.020);

    bigArm.setEncoder((int) Conversions.degreesToFalcon(Units.radiansToDegrees(bigArmSim.getAngleRads()), 80*3));
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(bigArmSim.getCurrentDrawAmps()));
    bigArmLigament2d.setAngle(Units.radiansToDegrees(bigArmSim.getAngleRads()));
    SmartDashboard.putNumber("bigArmSimPose", bigArm.getEncoder());

    lilArmSim.setInput(lilArm.getSpeed() * RobotController.getBatteryVoltage());
    lilArmSim.update(0.020);
    lilArm.setEncoder((int)Conversions.degreesToFalcon(Units.radiansToDegrees(lilArmSim.getAngleRads()), 1));
    RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(lilArmSim.getCurrentDrawAmps()));
    lilArmlLigament2d.setAngle(Units.radiansToDegrees(lilArmSim.getAngleRads()));
    SmartDashboard.putNumber("lilArmSimPose",lilArm.getEncoder());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
