// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommands {
    Elevator elevatorSubsystem;

    public ElevatorCommands(Elevator elevator) {
        this.elevatorSubsystem = elevator;
    }

    public Command intakePosition() {
        Command command = elevatorSubsystem.goToPositionCommand(Meters.of(0.005));
        command.setName("Elevator Intake Position");
        return command;
    }

    public Command home() {
        Command command = elevatorSubsystem.goToPositionCommand(Meters.of(0.21));
        command.setName("Elevator Idle Position");
        return command;
    }

    public Command positionLevel2() {
        Command command = elevatorSubsystem.goToPositionCommand(Meters.of(0.15));
        command.setName("Elevator Score Level 2");
        return command;
    }

    public Command positionLevel3() {
        Command command = elevatorSubsystem.goToPositionCommand(Meters.of(0.263));
        command.setName("Elevator Score Level 3");
        return command;
    }

    public Command positionLevel4() {
        Command command = elevatorSubsystem.goToPositionCommand(Meters.of(0.65));
        command.setName("Elevator Score Level 4");
        return command;
    }
}
