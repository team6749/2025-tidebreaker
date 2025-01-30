// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ElevatorCommands;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.swerve.SwerveDrive;

@Logged
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

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
    // Create subsystems
    swerveSubsystem = new SwerveDrive();
    localizationSubsystem = new Localization(swerveSubsystem);
    elevatorSubsystem = new Elevator();
    elevatorCommands = new ElevatorCommands(elevatorSubsystem);


    // Bind Path planner commands
    if(Robot.isSimulation()) {
      // For now, simulate a delay on intake
      NamedCommands.registerCommand("intake", new WaitCommand(1.5).andThen(intakeCommand()));
    } else {
      NamedCommands.registerCommand("intake", intakeCommand());
    }
    NamedCommands.registerCommand("score_l3", scoreLevel3());

    // Init Pathplanner
    try {
      RobotConfig config = RobotConfig.fromGUISettings();

      // Configure AutoBuilder last
      AutoBuilder.configure(
          localizationSubsystem::getRobotPose, // Robot pose supplier
          localizationSubsystem::resetPose, // Method to reset odometry (will be called if your auto has a starting
                                            // pose)
          // TODO this might be better to derive from the estimated robot pose
          swerveSubsystem::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> swerveSubsystem.runChassisSpeeds(speeds), // Method that will drive the robot given
                                                                              // ROBOT RELATIVE ChassisSpeeds. Also
                                                                              // optionally outputs individual module
                                                                              // feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                          // holonomic
                                          // drive trains
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
          ),
          config, // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          swerveSubsystem);

    } catch (Exception e) {
      e.printStackTrace();
    }

    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // Configure defaults
    swerveSubsystem.setDefaultCommand(swerveSubsystem.basicDriveCommand(controller));

    // Bind buttons and trigger behavior
    bButton.onTrue(intakeCommand());
    aButton.onTrue(elevatorCommands.positionLevel3());
  }

  public Command scoreLevel3() {
    Command command = Commands.sequence(elevatorCommands.idlePosition(),
        /* Arm Intake angle command, */ elevatorCommands.positionLevel3(), elevatorCommands.idlePosition());
    command.setName("Score Level 2");
    return command;
  }

  public Command intakeCommand() {
    Command command = Commands.sequence(elevatorCommands.idlePosition(),
        /* Arm Intake angle command, */ elevatorCommands.intakePosition(), elevatorCommands.idlePosition());
    command.setName("Intake Command");
    return command;
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
