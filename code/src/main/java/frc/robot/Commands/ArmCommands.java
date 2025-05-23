// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConstrainedArmSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmCommands {
    ConstrainedArmSubsystem armSubsystem;

    public ArmCommands(ConstrainedArmSubsystem arm) {
        this.armSubsystem = arm;
    }
    
    public Command removeAlgae() {
        Command command = armSubsystem.runVoltsCommand(Volts.of(-1));
        return command;
    }

    public Command intakePosition() {//.227 angle
        Command command = armSubsystem.goToPositionCommand(Degrees.of(-61.707)); 
        command.setName("Arm Intake Position");
        return command;
    }

    public Command Home() {
        Command command = armSubsystem.goToPositionCommand(Degrees.of(-90));
        command.setName("Arm Idle Position");
        return command;
    }

    public Command positionLevel1() {
        Command command = armSubsystem.goToPositionCommand(Degrees.of(-90)); //random values
        command.setName("Arm Score Level 1");
        return command;
    }

    public Command positionLevel2() {
        Command command = armSubsystem.goToPositionCommand(Degrees.of(-80)); //random values
        command.setName("Arm Score Level 2");
        return command;
    }

    public Command positionLevel3() {
        Command command = armSubsystem.goToPositionCommand(Degrees.of(50));
        command.setName("Arm Score Level 3");
        return command;
    }

    public Command positionLevel4() {
        Command command = armSubsystem.goToPositionCommand(Degrees.of(82)); //53 old value
        command.setName("Arm Score Level 4");
        return command;
    }

    public Command score() {
        Command command = armSubsystem.runVoltsCommand(Volts.of(-1.5)).withTimeout(1);
        command.setName("Arm Score");
        return command;
    }

}
