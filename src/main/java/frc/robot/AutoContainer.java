// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.PrimoShuffleboard;
import frc.robot.autonomous.CommandSelector;
import frc.robot.commands.autoCommands.TwoPiece;
import frc.robot.subsystems.BigArm;
import frc.robot.subsystems.Gripper;
import frc.robot.subsystems.LilArm;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class AutoContainer {
    private CommandSelector autoSelector;
    private Map<String, Command> autoPaths;

    public AutoContainer(Swerve swerve, Gripper gripper, BigArm bigArm, LilArm lilArm){
        this.autoPaths = new HashMap<String, Command>(); 
        this.autoPaths.put("blueConeCube", new TwoPiece(true, false, true, bigArm, lilArm, gripper, swerve));
        this.autoSelector = new CommandSelector(autoPaths, PrimoShuffleboard.getInstance().getCompTabTitle());
    }

    public Command getAutonomousCommand() {
        return this.autoSelector.getCommand();
      }
}
    