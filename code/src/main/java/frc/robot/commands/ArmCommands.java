package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class ArmCommands {
    
    Arm arm;

    public ArmCommands (Arm arm) {
        this.arm = arm;
    }

    public Command intakePosition () {
        Command command = arm.goToPositionCommand(Degrees.of(-90));
        command.setName("Arm Intake Position");
        return command;
    }

    public Command scoreLevel2And3 () {
        Command command = arm.goToPositionCommand(Degrees.of(0));
        command.setName("Arm Score Position 2-3");
        return command;
    }

    public Command positionLevel4 () {
        Command command = arm.goToPositionCommand(Degrees.of(35));
        command.setName("Arm Score Level 4");
        return command;
    }
}
