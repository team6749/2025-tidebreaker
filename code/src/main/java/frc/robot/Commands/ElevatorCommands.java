// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorCommands {
    Elevator elevatorSubsystem;

    public static final LinearVelocity attackVelocity = MetersPerSecond.of(-0.35);

    public ElevatorCommands(Elevator elevator) {
        this.elevatorSubsystem = elevator;
    }

    public Command intakeAction() {
        Command command = 
                elevatorSubsystem.goToPositionCommand(Meters.of(0.288));
        command.setName("Elevator Intake");
        return command;
    }

    public Command home() {
        Command command = elevatorSubsystem.goToPositionCommand(Meters.of(0.24));
        command.setName("Elevator Idle Position");
        return command;
    }

    public Command positionLevel1() {
        Command command = elevatorSubsystem.goToPositionCommand(Meters.of(0.062));
        command.setName("Elevator Score Level 1");
        return command;
    }
    
    public Command positionLevel2() {
        Command command = elevatorSubsystem.goToPositionCommand(Meters.of(0.197));
        command.setName("Elevator Score Level 2");
        return command;
    }

    public Command positionLevel3() {
        Command command = elevatorSubsystem.goToPositionCommand(Meters.of(0.084));
        command.setName("Elevator Score Level 3");
        return command;
    }

    public Command positionLevel4() {
        Command command = elevatorSubsystem.goToPositionCommand(Meters.of(0.625));
        command.setName("Elevator Score Level 4");
        return command;
    }

    public Command reHome() {
        Command command = elevatorSubsystem.reHome();
        command.setName("Elevator Score Level 4");
        return command;
    }
}
