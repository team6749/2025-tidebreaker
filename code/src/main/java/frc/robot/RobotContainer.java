// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Volts;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.Elevator;

@Logged
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

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

    try {
      RobotConfig config = RobotConfig.fromGUISettings();
      // Configure AutoBuilder last
      AutoBuilder.configure(
          localizationSubsystem::getRobotPose, // Robot pose supplier
          localizationSubsystem::resetPose, // Method to reset odometry (will be called if your auto has a starting
                                            // pose)
          swerveSubsystem::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speeds, feedforwards) -> swerveSubsystem.runChassisSpeeds(speeds), // Method that will drive the robot given
                                                                              // ROBOT RELATIVE ChassisSpeeds. Also
                                                                              // optionally outputs individual module
                                                                              // feedforwards
          new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for
                                          // holonomic drive trains
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
          swerveSubsystem // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("autoChooser", autoChooser);

    configureBindings();

    // elevatorTest();
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.basicDriveCommand(controller, localizationSubsystem));

    rightBumper.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.intake));
    y.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l1));
    b.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l2));
    a.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l3));
    x.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l4));
  }

  private void elevatorTest() {
    a.whileTrue(Commands.repeatingSequence(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l3),
        elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l1)));
    y.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l1));
    // y.whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(0.5)));
    b.whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(0.3)));
    // a.whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(0.1)));
    x.whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(0.2)));
    rightBumper.whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(-0.3)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
