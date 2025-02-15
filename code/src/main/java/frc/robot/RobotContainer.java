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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmSample;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.Elevator;

@Logged
public class RobotContainer {

  SwerveDrive swerveSubsystem;
  Arm arm = new Arm();
  //ArmSample armSample = new ArmSample();
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
    //elevatorTest();
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.basicDriveCommand(controller));
    a.onTrue(arm.goToPositionArm(Radians.of(0.6)));
    y.onTrue(arm.goToPositionArm(Radians.of(0.8)));
    b.onTrue(arm.goToPositionArm(Radians.of(Math.PI / 2)));
    x.onTrue(arm.goToPositionArm(Radians.of(Math.PI * 3 / 2))); //find real values
    rightBumper.onTrue(arm.goToPositionArm(Radians.of(0)));
    //swerveSubsystem.setDefaultCommand(swerveSubsystem.basicDriveCommand(controller));
    
    rightBumper.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.intake));
    y.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l1));
    b.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l2));
    a.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l3));
    x.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l4));
    //swerveSubsystem.setDefaultCommand(swerveSubsystem.testModuleSpeeds(new SwerveModuleState(MetersPerSecond.of(2),Rotation2d.kZero)));
  }
  private void elevatorTest() {
    a.whileTrue(Commands.repeatingSequence(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l3),elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l1)));
    y.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l1));
    b.whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(1)));
    x.whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(0.5)));
    rightBumper.whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(-0.3)));
  }

  public Command getAutonomousCommand() {
    return swerveSubsystem.constantChassisSpeedsCommand(
        new ChassisSpeeds(2, 0, 2));
  }


}
