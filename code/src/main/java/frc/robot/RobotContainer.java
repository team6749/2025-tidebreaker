// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.ArmCommands;
import frc.robot.Commands.ElevatorCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.Elevator;

@Logged
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  SwerveDrive swerveSubsystem;
  Arm arm = new Arm();
  // ArmSample armSample = new ArmSample();
  Localization localizationSubsystem;
  Elevator elevatorSubsystem = new Elevator();
  ElevatorCommands elevatorCommands = new ElevatorCommands(elevatorSubsystem);
  ArmCommands armCommands = new ArmCommands(arm);

  XboxController controller = new XboxController(0);
  XboxController controller2 = new XboxController(1);
  // PS5Controller controller2 = new PS5Controller(1);
  JoystickButton a = new JoystickButton(controller, 1);
  JoystickButton x = new JoystickButton(controller, 3);
  JoystickButton b = new JoystickButton(controller, 2);
  JoystickButton y = new JoystickButton(controller, 4);
  JoystickButton rightBumper = new JoystickButton(controller2, 5);
  JoystickButton leftBumper = new JoystickButton(controller2, 6);
  DoubleSupplier rightTrigger = () -> controller2.getRawAxis(3);
  DoubleSupplier leftTrigger = () -> controller2.getRawAxis(2);

  public RobotContainer() {
    swerveSubsystem = new SwerveDrive();
    localizationSubsystem = new Localization(swerveSubsystem);

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

    // configureBindings();
    // elevatorTest();
    // armTest();
    coralSubsystemTest();
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.basicDriveCommand(controller, localizationSubsystem));

    new Trigger(() -> leftTrigger.getAsDouble() > 0.5).whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(-1))); // find
                                                                                                                      // real
                                                                                                                      // values
    new Trigger(() -> rightTrigger.getAsDouble() > 0.5).whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(1)));
    leftBumper.whileTrue(arm.runOpenLoopCommand(Volts.of(0.7)));
    rightBumper.whileTrue(arm.runOpenLoopCommand(Volts.of(-0.7)));
    b.whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(-0.3)));
    x.whileTrue(arm.runOpenLoopCommand(Volts.of(0.3)));
  }

  private void elevatorTest() {
    a.whileTrue(Commands.repeatingSequence(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l3),
        elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l1)));
    y.whileTrue(elevatorSubsystem.goToPositionCommand(Constants.ElevatorSetPoints.l1));
    b.whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(1)));
    x.whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(0.5)));
    rightBumper.whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(-0.3)));
  }

  private void armTest() {
    a.whileTrue(arm.runOpenLoopCommand(Volts.of(2)));
    y.whileTrue(arm.runOpenLoopCommand(Volts.of(0.1)));
    x.whileTrue(arm.runOpenLoopCommand(Volts.of(-2))); // find real values
  }

  private void coralSubsystemTest() {
    a.whileTrue(Home());
    b.whileTrue(moveToLevel2());
    x.whileTrue(moveToLevel3());
    y.whileTrue(moveToLevel4());
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public Command Home() {
    Command command = Commands.sequence(
      armCommands.Home(),
        elevatorCommands.Home());
    command.setName("Home");
    return command;
  }

  public Command moveToLevel2() {
    Command command = Commands.sequence(
        elevatorCommands.positionLevel2(),
        armCommands.positionLevel2());
    command.setName("Level 2");
    return command;
  }

  public Command moveToLevel3() {
    Command command = Commands.sequence(
        elevatorCommands.positionLevel3(),
        armCommands.positionLevel3());
    command.setName("Level 3");
    return command;
  }

  public Command moveToLevel4() {
    Command command = Commands.sequence(
        elevatorCommands.positionLevel4(),
        armCommands.positionLevel4());
    command.setName("Level 4");
    return command;
  }

  public Command Intake() {
    Command command = Commands.sequence(
        elevatorCommands.intakePosition(),
        armCommands.intakePosition());
    command.setName("intake Coral");
    return command;
  }

  public Command Score() {
    Command command = Commands.sequence(
        armCommands.score());
    command.setName("Score");
    return command;
  }
}
