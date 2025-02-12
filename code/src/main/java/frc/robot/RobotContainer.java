// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.swerve.SwerveDrive;

@Logged
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  SwerveDrive swerveSubsystem;
  Localization localizationSubsystem;

  XboxController controller = new XboxController(0);

  public RobotContainer() {
    swerveSubsystem = new SwerveDrive();
    localizationSubsystem = new Localization(swerveSubsystem);

    try{
      RobotConfig config = RobotConfig.fromGUISettings();
      // Configure AutoBuilder last
    AutoBuilder.configure(
      localizationSubsystem::getRobotPose, // Robot pose supplier
      localizationSubsystem::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
      swerveSubsystem::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
      (speeds, feedforwards) -> swerveSubsystem.runChassisSpeeds(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
      new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
      ),
      config, // The robot configuration
      () -> {
        // Boolean supplier that controls when the path will be mirrored for the red alliance
        // This will flip the path being followed to the red side of the field.
        // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      },
      swerveSubsystem // Reference to this subsystem to set requirements
);
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    
  
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("autoChooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.basicDriveCommand(controller));
    //swerveSubsystem.setDefaultCommand(swerveSubsystem.testModuleSpeeds(new SwerveModuleState(MetersPerSecond.of(2),Rotation2d.kZero)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    }
}
