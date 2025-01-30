// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.swerve.SwerveDrive;

@Logged
public class RobotContainer {

  SwerveDrive swerveSubsystem;
  Localization localizationSubsystem;
  Elevator elevatorSubsystem;
  ElevatorCommands elevatorCommands;

  XboxController controller = new XboxController(0);

  Trigger aButton = new Trigger(() -> controller.getBButton());
  Trigger bButton = new Trigger(() -> controller.getAButton());
  Trigger xButton = new Trigger(() -> controller.getXButton());
  Trigger yButton = new Trigger(() -> controller.getYButton());
  
  public RobotContainer() {
    swerveSubsystem = new SwerveDrive();
    localizationSubsystem = new Localization(swerveSubsystem);
    elevatorSubsystem = new Elevator();
    elevatorCommands = new ElevatorCommands(elevatorSubsystem);



    configureBindings();
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.basicDriveCommand(controller));
    
    bButton.onTrue(intakeCommand());
    aButton.onTrue(elevatorCommands.positionLevel3());
  }

  public Command intakeCommand () {
    Command command = Commands.sequence(elevatorCommands.idlePosition(), /* Arm Intake angle command, */ elevatorCommands.intakePosition(), elevatorCommands.idlePosition());
    command.setName("Intake Command");
    return command;
}


  public Command getAutonomousCommand() {
    return Commands.repeatingSequence(elevatorSubsystem.goToPositionCommand(Meter.of(0.75)), elevatorSubsystem.goToPositionCommand(Meter.of(0.2)));
    // return swerveSubsystem.constantChassisSpeedsCommand(
    //     new ChassisSpeeds(2, 0, 2));
  }
}
