// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FileVersionException;

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
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmSample;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.Elevator;

@Logged
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;

  SwerveDrive swerveSubsystem;
  Arm arm = new Arm();
  //ArmSample armSample = new ArmSample();
  Localization localizationSubsystem;
  Elevator elevatorSubsystem;
  POICommands poiCommands; 

  private final Joystick topButtonBoard = new Joystick(Constants.kTopButtonBoardPort);
  private final Joystick bottomButtonBoard = new Joystick(Constants.kBottomButtonBoardPort);

  XboxController controller = new XboxController(0);
  XboxController controller2 = new XboxController(1);
  // PS5Controller controller2 = new PS5Controller(1);
  JoystickButton a = new JoystickButton(controller, 1);
  JoystickButton x = new JoystickButton(controller, 3);
  JoystickButton b = new JoystickButton(controller2, 2);
  JoystickButton y = new JoystickButton(controller2, 4);
  JoystickButton rightBumper = new JoystickButton(controller2, 5);
  JoystickButton leftBumper = new JoystickButton(controller2, 6);
  DoubleSupplier rightTrigger = () -> controller2.getRawAxis(3);
  DoubleSupplier leftTrigger =  () -> controller2.getRawAxis(2);

  JoystickButton buttonCoralJ = new JoystickButton(bottomButtonBoard, 1);
  JoystickButton buttonCoralI = new JoystickButton(bottomButtonBoard, 2);
  JoystickButton buttonCoralH = new JoystickButton(bottomButtonBoard, 3);
  JoystickButton buttonCoralG = new JoystickButton(bottomButtonBoard, 4);
  JoystickButton buttonCoralF = new JoystickButton(bottomButtonBoard, 5);
  JoystickButton bottomButton6 = new JoystickButton(bottomButtonBoard, 6);
  JoystickButton bottomButton7 = new JoystickButton(bottomButtonBoard, 7);
  JoystickButton buttonCoralE = new JoystickButton(bottomButtonBoard, 8);
  JoystickButton buttonCoralD = new JoystickButton(bottomButtonBoard, 9);
  JoystickButton buttonCoralC = new JoystickButton(bottomButtonBoard, 10);
  JoystickButton buttonCoralB = new JoystickButton(bottomButtonBoard, 11);
  JoystickButton buttonCoralA = new JoystickButton(bottomButtonBoard, 12);

  JoystickButton topButton1 = new JoystickButton(topButtonBoard, 1);
  JoystickButton topButton2 = new JoystickButton(topButtonBoard, 2);
  JoystickButton topButton3 = new JoystickButton(topButtonBoard, 3);
  JoystickButton topButton4 = new JoystickButton(topButtonBoard, 4);
  JoystickButton topButton5 = new JoystickButton(topButtonBoard, 5);
  JoystickButton topButton6 = new JoystickButton(topButtonBoard, 6);
  JoystickButton topButton7 = new JoystickButton(topButtonBoard, 7);
  JoystickButton topButton8 = new JoystickButton(topButtonBoard, 8);
  JoystickButton buttonRightIntake = new JoystickButton(topButtonBoard, 9);
  JoystickButton buttonLeftIntake = new JoystickButton(topButtonBoard, 10);
  JoystickButton buttonCoralL = new JoystickButton(topButtonBoard, 11);
  JoystickButton buttonCoralK = new JoystickButton(topButtonBoard, 12);

  public RobotContainer() {
    swerveSubsystem = new SwerveDrive();
    localizationSubsystem = new Localization(swerveSubsystem);
    elevatorSubsystem = new Elevator();
    poiCommands = new POICommands(swerveSubsystem);
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
    //elevatorTest();
    //armTest();
    try {
      autoAlignTest();
    } catch (FileVersionException | IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.basicDriveCommand(controller, localizationSubsystem));

    new Trigger(() -> leftTrigger.getAsDouble() > 0.5).whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(-1))); //find real values
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
    x.whileTrue(arm.runOpenLoopCommand(Volts.of(-2))); //find real values
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void autoAlignTest() throws FileVersionException, IOException, ParseException {
    buttonCoralA.whileTrue(poiCommands.pathToCoralA());
    buttonCoralB.whileTrue(poiCommands.pathToCoralB());
    buttonCoralC.whileTrue(poiCommands.pathToCoralC());
    buttonCoralD.whileTrue(poiCommands.pathToCoralD());
    buttonCoralE.whileTrue(poiCommands.pathToCoralE());
    buttonCoralF.whileTrue(poiCommands.pathToCoralF());
    buttonCoralG.whileTrue(poiCommands.pathToCoralG());
    buttonCoralH.whileTrue(poiCommands.pathToCoralH());
    buttonCoralI.whileTrue(poiCommands.pathToCoralI());
    buttonCoralJ.whileTrue(poiCommands.pathToCoralJ());
    buttonCoralK.whileTrue(poiCommands.pathToCoralK());
    buttonCoralL.whileTrue(poiCommands.pathToCoralL());
    buttonLeftIntake.whileTrue(poiCommands.pathToLeftIntake());
    buttonRightIntake.whileTrue(poiCommands.pathToRightIntake());
  }
}
