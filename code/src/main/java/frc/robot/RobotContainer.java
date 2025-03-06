// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Commands.ArmCommands;
import frc.robot.Commands.ElevatorCommands;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.Elevator;

@Logged
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  ClimberSubsystem climberSubsystem;
  SwerveDrive swerveSubsystem;
  Arm arm;
  // ArmSample armSample = new ArmSample();
  Localization localizationSubsystem;
  Elevator elevatorSubsystem;
  ElevatorCommands elevatorCommands;
  ArmCommands armCommands;
  POICommands poiCommands;

  private final Joystick topButtonBoard = new Joystick(Constants.kTopButtonBoardPort);
  private final Joystick bottomButtonBoard = new Joystick(Constants.kBottomButtonBoardPort);

  XboxController controller = new XboxController(0);
  JoystickButton a = new JoystickButton(controller, 1);
  JoystickButton x = new JoystickButton(controller, 3);
  JoystickButton b = new JoystickButton(controller, 2);
  JoystickButton y = new JoystickButton(controller, 4);
  JoystickButton rightBumper = new JoystickButton(controller, 5);
  JoystickButton leftBumper = new JoystickButton(controller, 6);
  DoubleSupplier rightTrigger = () -> controller.getRawAxis(3);
  DoubleSupplier leftTrigger = () -> controller.getRawAxis(2);

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

  JoystickButton buttonScore = new JoystickButton(topButtonBoard, 1);
  JoystickButton buttonHome = new JoystickButton(topButtonBoard, 2);
  JoystickButton buttonIntake = new JoystickButton(topButtonBoard, 3);
  JoystickButton buttonL4 = new JoystickButton(topButtonBoard, 4);
  JoystickButton buttonL3 = new JoystickButton(topButtonBoard, 5);
  JoystickButton buttonTop6 = new JoystickButton(topButtonBoard, 6);
  JoystickButton buttonTop7 = new JoystickButton(topButtonBoard, 7);
  JoystickButton buttonL2 = new JoystickButton(topButtonBoard, 8);
  JoystickButton buttonRightIntake = new JoystickButton(topButtonBoard, 9);
  JoystickButton buttonLeftIntake = new JoystickButton(topButtonBoard, 10);
  JoystickButton buttonCoralL = new JoystickButton(topButtonBoard, 11);
  JoystickButton buttonCoralK = new JoystickButton(topButtonBoard, 12);

  public RobotContainer() {
    swerveSubsystem = new SwerveDrive();
    arm = new Arm();
    localizationSubsystem = new Localization(swerveSubsystem);
    elevatorSubsystem = new Elevator();
    poiCommands = new POICommands(swerveSubsystem);
    elevatorCommands = new ElevatorCommands(elevatorSubsystem);
    armCommands = new ArmCommands(arm);
    climberSubsystem = new ClimberSubsystem();
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

    

    // coralSubsystemTest();
    //configureBindings();
    //elevatorTest();
    //armTest();
    //sysIDSwerve();
    sysIDElevator();
    //sysIDArm();
    try {
      autoAlignTest();
    } catch (FileVersionException | IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(swerveSubsystem.basicDriveCommand(controller, localizationSubsystem));

    // Add Rest Pose Command
    SmartDashboard.putData("Reset Pose", Commands.runOnce(() -> {
      localizationSubsystem.resetPose(Pose2d.kZero);
    }, localizationSubsystem));

    try {
      SmartDashboard.putData("Align/A", poiCommands.pathToCoralA());
      SmartDashboard.putData("Align/B", poiCommands.pathToCoralB());
      SmartDashboard.putData("Align/C", poiCommands.pathToCoralC());
      SmartDashboard.putData("Align/D", poiCommands.pathToCoralD());
      SmartDashboard.putData("Align/E", poiCommands.pathToCoralE());
      SmartDashboard.putData("Align/F", poiCommands.pathToCoralF());
      SmartDashboard.putData("Align/G", poiCommands.pathToCoralG());
      SmartDashboard.putData("Align/H", poiCommands.pathToCoralH());
      SmartDashboard.putData("Align/I", poiCommands.pathToCoralI());
      SmartDashboard.putData("Align/J", poiCommands.pathToCoralJ());
      SmartDashboard.putData("Align/K", poiCommands.pathToCoralK());
      SmartDashboard.putData("Align/L", poiCommands.pathToCoralL());
      SmartDashboard.putData("Align/IntakeLeft", poiCommands.pathToLeftIntake());
      SmartDashboard.putData("Align/IntakeRight", poiCommands.pathToRightIntake());
    } catch (FileVersionException | IOException | ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

    rightBumper.whileTrue(climberSubsystem.climbCommand());
    leftBumper.whileTrue(climberSubsystem.unclimbCommand());
    buttonHome.whileTrue(home());
    buttonL2.whileTrue(moveToLevel2());
    buttonL3.whileTrue(moveToLevel3());
    buttonL4.whileTrue(moveToLevel4());
    buttonIntake.whileTrue(intake());
    buttonScore.whileTrue(score());
    a.whileTrue(climberSubsystem.climbCommand());
    b.whileTrue(climberSubsystem.unclimbCommand());
  }

  @SuppressWarnings("unused")
  private void elevatorTest() {
    a.whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(-0.3)));
    b.whileTrue(elevatorSubsystem.runOpenLoopCommand(Volts.of(0.3)));
  }

  @SuppressWarnings("unused")
  private void armTest() {
    a.whileTrue(arm.runOpenLoopCommand(Volts.of(-0.5), Radians.of(0.1)));
    b.whileTrue(arm.runOpenLoopCommand(Volts.of(0.5), Radians.of(1)));
    // b.whileTrue(arm.runOpenLoopCommand(Volts.of(2), Radians.of(1)));
    // x.whileTrue(arm.runOpenLoopCommand(Volts.of(-0.5), Radians.of(1.3)));
  }

  @SuppressWarnings("unused")
  private void coralSubsystemTest() {
    buttonHome.whileTrue(home());
    buttonL2.whileTrue(moveToLevel2());
    buttonL3.whileTrue(moveToLevel3());
    buttonL4.whileTrue(moveToLevel4());
    buttonIntake.whileTrue(intake());
    buttonScore.whileTrue(score());
  }

  private void sysIDSwerve() {
    a.whileTrue(swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    b.whileTrue(swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    x.whileTrue(swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    y.whileTrue(swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }
  private void sysIDElevator() {
    a.whileTrue(elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    b.whileTrue(elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    //x.whileTrue(elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //y.whileTrue(elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }
  private void sysIDArm() {
    a.whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    b.whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    //x.whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    //y.whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
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

  public Command home() {
    Command command = Commands.parallel(
        armCommands.Home(),
        elevatorCommands.home());
    command.setName("Home");
    return command;
  }

  public Command moveToLevel2() {
    Command command = Commands.parallel(
        elevatorCommands.positionLevel2(),
        armCommands.positionLevel2());
    command.setName("Level 2");
    return command;
  }

  public Command moveToLevel3() {
    Command command = Commands.parallel(
        elevatorCommands.positionLevel3(),
        armCommands.positionLevel3());
    command.setName("Level 3");
    return command;
  }

  public Command moveToLevel4() {
    Command command = Commands.parallel(
        elevatorCommands.positionLevel4(),
        armCommands.positionLevel4());
    command.setName("Level 4");
    return command;
  }

  public Command intake() {
    Command command = Commands.sequence(Commands.parallel(
        elevatorCommands.home(),
        armCommands.intakePosition()), elevatorCommands.intakePosition(), elevatorCommands.home());
    command.setName("intake Coral");
    return command;
  }

  public Command score() {
    Command command = Commands.parallel(
        armCommands.score());
    command.setName("Score");
    return command;
  }
  
}
