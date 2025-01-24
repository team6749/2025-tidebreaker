// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.swerve.SwerveBase;
import frc.robot.subsystems.swerve.SwerveSim;

@Logged
public class RobotContainer {

  SwerveBase swerveSubsystem;
  Localization localizationSubsystem;

  XboxController controller = new XboxController(0);

  public RobotContainer() {
    if(Robot.isSimulation()) {
      swerveSubsystem = new SwerveSim();
    } else {
      // TODO implement real swerve subsystem
    }
    localizationSubsystem = new Localization(swerveSubsystem);

    configureBindings();
  }

  private void configureBindings() {
    
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
