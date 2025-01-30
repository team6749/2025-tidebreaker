package frc.robot.commands;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorCommands {
    
    Elevator elevator;

    public ElevatorCommands (Elevator elevator) {
        this.elevator = elevator;
    }

    public Command intakePosition () {
        Command command = elevator.goToPositionCommand(Meters.of(0.05));
        command.setName("Elevator Intake Position");
        return command;
    }

    public Command idlePosition () {
        Command command = elevator.goToPositionCommand(Meters.of(0.2));
        command.setName("Elevator Idle Position");
        return command;
    }

    public Command positionLevel2 () {
        Command command = elevator.goToPositionCommand(Meters.of(0.3));
        command.setName("Elevator Score Level 2");
        return command;
    }

    public Command positionLevel3 () {
        Command command = elevator.goToPositionCommand(Meters.of(0.8));
        command.setName("Elevator Score Level 3");
        return command;
    }

    public Command positionLevel4 () {
        Command command = elevator.goToPositionCommand(Meters.of(1.2));
        command.setName("Elevator Score Level 4");
        return command;
    }

}
