// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.Elevator;

@Logged
public class RobotContainer {

  SwerveDrive swerveSubsystem;
  Localization localizationSubsystem;
  Elevator elevatorSubsystem;

  XboxController controller = new XboxController(0);
  JoystickButton a = new JoystickButton(controller, 1);
  JoystickButton b = new JoystickButton(controller, 2);
  JoystickButton x = new JoystickButton(controller, 3);
  JoystickButton y = new JoystickButton(controller, 4);
  JoystickButton rightBumper = new JoystickButton(controller, 6);

  public RobotContainer() {
    swerveSubsystem = new SwerveDrive();
    localizationSubsystem = new Localization(swerveSubsystem);
    elevatorSubsystem = new Elevator();


    configureBindings();
  }

  private void configureBindings() {
    //swerveSubsystem.setDefaultCommand(swerveSubsystem.basicDriveCommand(controller));
    
    // y.onTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(0.5)));
    // b.onTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(0.3)));
    // a.onTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(0.1)));
    // x.onTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(0.2)));
    // rightBumper.onTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(-0.3)));
    rightBumper.onTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.intake));
    y.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l1));
    b.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l2));
    a.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l3));
    x.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l4));
    //swerveSubsystem.setDefaultCommand(swerveSubsystem.testModuleSpeeds(new SwerveModuleState(MetersPerSecond.of(2),Rotation2d.kZero)));
  }

  public Command getAutonomousCommand() {
    return swerveSubsystem.constantChassisSpeedsCommand(
        new ChassisSpeeds(2, 0, 2));
  }


}
