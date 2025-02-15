// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import java.util.Queue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.enums.DriveOrientation;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.swerve.SwerveDrive;

@Logged
public class RobotContainer {
//  private final SendableChooser<DriveOrientation> orientationChooser = new SendableChooser<>();

  SwerveDrive swerveSubsystem;
  Localization localizationSubsystem;

  XboxController controller = new XboxController(0);

  public RobotContainer() {
    swerveSubsystem = new SwerveDrive();
    localizationSubsystem = new Localization(swerveSubsystem);



    configureBindings();

    // orientationChooser.setDefaultOption("Field Orientation", DriveOrientation.FieldOriented);
    // orientationChooser.addOption("Robot Orientation", DriveOrientation.RobotOriented);
    // SmartDashboard.putData("Robot Orientation Mode", orientationChooser);
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.basicDriveCommand(controller, localizationSubsystem));

  }
  
  public Command getAutonomousCommand() {
    return swerveSubsystem.constantChassisSpeedsCommand(
        new ChassisSpeeds(2, 0, 2));
  }
}
