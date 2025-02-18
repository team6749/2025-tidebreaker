// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCommands {
    Arm armSubsystem;

    public ArmCommands(Arm arm) {
        this.armSubsystem = arm;
    }

    public Command intakePosition() {
        Command command = armSubsystem.goToPositionArm(Radians.of(-Math.PI / 2));
        command.setName("Arm Intake Position");
        return command;
    }

    public Command Home() {
        Command command = armSubsystem.goToPositionArm(Radians.of(-Math.PI / 2));
        command.setName("Arm Idle Position");
        return command;
    }

    public Command positionLevel2() {
        Command command = armSubsystem.goToPositionArm(Radians.of(0.7));
        command.setName("Arm Score Level 2");
        return command;
    }

    public Command positionLevel3() {
        Command command = armSubsystem.goToPositionArm(Radians.of(1));
        command.setName("Arm Score Level 3");
        return command;
    }

    public Command positionLevel4() {
        Command command = armSubsystem.goToPositionArm(Radians.of(1.57));
        command.setName("Arm Score Level 4");
        return command;
    }

    public Command score() {
        Command command = armSubsystem.goToPositionArm(Radians.of(-Math.PI * 5 / 3));
        command.setName("Arm Score Level 4");
        return command;
    }
}
