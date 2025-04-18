// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.util.function.DoubleSupplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
import frc.robot.subsystems.ConstrainedArmSubsystem;
import frc.robot.Commands.POICommands;
import frc.robot.subsystems.ActiveClawSubsystem;
import frc.robot.subsystems.AlgaeSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.Localization;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.Elevator;

@Logged
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  ActiveClawSubsystem clawSubsystem;
  ClimberSubsystem climberSubsystem;
  SwerveDrive swerveSubsystem;
  ConstrainedArmSubsystem armSubsystem;
  AlgaeSubsystem algaeSubsystem;
  Localization localizationSubsystem;
  Elevator elevatorSubsystem;
  ElevatorCommands elevatorCommands;
  ArmCommands armCommands;
  POICommands POICommands;

  private final Joystick topButtonBoard = new Joystick(Constants.kTopButtonBoardPort);
  private final Joystick bottomButtonBoard = new Joystick(Constants.kBottomButtonBoardPort);

  XboxController controller = new XboxController(0);
  JoystickButton a = new JoystickButton(controller, 1);
  JoystickButton x = new JoystickButton(controller, 3);
  JoystickButton b = new JoystickButton(controller, 2);
  JoystickButton y = new JoystickButton(controller, 4);
  JoystickButton startButton = new JoystickButton(controller, 8);
  JoystickButton rightBumper = new JoystickButton(controller, 5);
  JoystickButton leftBumper = new JoystickButton(controller, 6);
  DoubleSupplier rightTrigger = () -> controller.getRawAxis(3);
  DoubleSupplier leftTrigger = () -> controller.getRawAxis(2);

  JoystickButton buttonLevel3 = new JoystickButton(bottomButtonBoard, 1);
  JoystickButton buttonCoralC = new JoystickButton(bottomButtonBoard, 2);
  JoystickButton buttonIntake = new JoystickButton(bottomButtonBoard, 3);
  JoystickButton buttonLevel2 = new JoystickButton(bottomButtonBoard, 4);
  JoystickButton buttonRightIntake = new JoystickButton(bottomButtonBoard, 5);
  JoystickButton buttonL6 = new JoystickButton(bottomButtonBoard, 6);
  JoystickButton buttonScore = new JoystickButton(bottomButtonBoard, 7);
  JoystickButton buttonCoralE = new JoystickButton(bottomButtonBoard, 8);
  JoystickButton buttonL10 = new JoystickButton(bottomButtonBoard, 9);
  JoystickButton buttonCoralD = new JoystickButton(bottomButtonBoard, 10);
  JoystickButton buttonLevel4 = new JoystickButton(bottomButtonBoard, 11);
  JoystickButton buttonHome = new JoystickButton(bottomButtonBoard, 12);

  JoystickButton buttonCoralI = new JoystickButton(topButtonBoard, 1);
  JoystickButton buttonCoralG = new JoystickButton(topButtonBoard, 2);
  JoystickButton buttonCoralA = new JoystickButton(topButtonBoard, 3);
  JoystickButton buttonCoralJ = new JoystickButton(topButtonBoard, 4);
  JoystickButton buttonCoralK = new JoystickButton(topButtonBoard, 5);
  JoystickButton buttonLeftIntake = new JoystickButton(topButtonBoard, 6);
  JoystickButton buttonL9 = new JoystickButton(topButtonBoard, 7);
  JoystickButton buttonL8 = new JoystickButton(topButtonBoard, 8);
  JoystickButton buttonCoralF = new JoystickButton(topButtonBoard, 9);
  JoystickButton buttonCoralH = new JoystickButton(topButtonBoard, 10);
  JoystickButton buttonCoralB = new JoystickButton(topButtonBoard, 11);
  JoystickButton buttonCoralL = new JoystickButton(topButtonBoard, 12);

  public RobotContainer() {
    swerveSubsystem = new SwerveDrive();
    armSubsystem = new ConstrainedArmSubsystem();
    clawSubsystem = new ActiveClawSubsystem(armSubsystem);
    algaeSubsystem = new AlgaeSubsystem();
    localizationSubsystem = new Localization(swerveSubsystem);
    elevatorSubsystem = new Elevator();
    POICommands = new POICommands(swerveSubsystem);
    elevatorCommands = new ElevatorCommands(elevatorSubsystem);
    armCommands = new ArmCommands(armSubsystem);
    climberSubsystem = new ClimberSubsystem();

    NamedCommands.registerCommand("home", home());
    NamedCommands.registerCommand("intake", intakeAuto());
    NamedCommands.registerCommand("wait_for_coral", Commands.idle(elevatorSubsystem)
        .until(() -> clawSubsystem.hasCoral()).withTimeout(Robot.isSimulation() ? 1.25 : 2.5)); // less trust on stall detect. At least for the first 3 corals where the ls isn't broken
    NamedCommands.registerCommand("score", clawSubsystem.clawShoot().withTimeout(Seconds.of(0.5)));
    NamedCommands.registerCommand("l2", moveToLevel2());
    NamedCommands.registerCommand("l3", moveToLevel3());
    NamedCommands.registerCommand("l4", moveToLevel4());
    NamedCommands.registerCommand("remove_algae", removeAlgae());

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
              new PIDConstants(4.0, 0.0, 0.0), // Translation PID constants
              new PIDConstants(4.0, 0.0, 0.0) // Rotation PID constants
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

    coralSubsystemTest();
    configureBindings();
    clawTest();
    algaeTest();
    // elevatorTest();
    // armTest();
    // sysIDSwerve();
    // sysIDElevator();
    // sysIDArm();

    try {
      autoAlignTest();
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }
  }

  private void configureBindings() {
    clawSubsystem.setDefaultCommand(clawSubsystem.clawIdleState());
    swerveSubsystem.setDefaultCommand(swerveSubsystem.basicDriveCommand(controller, localizationSubsystem));

    buttonHome.whileTrue(intakeTeleop());
    buttonLevel2.whileTrue(moveToLevel2());
    buttonLevel3.whileTrue(moveToLevel3());
    buttonLevel4.whileTrue(moveToLevel4());
    buttonIntake.whileTrue(removeAlgae());
    buttonScore.whileTrue(clawSubsystem.clawShoot());

    a.whileTrue(armSubsystem.runVoltsCommand(Volts.of(1)));
    b.whileTrue(elevatorSubsystem.runVoltsCommand(Volts.of(1)));
    x.whileTrue(elevatorSubsystem.runVoltsCommand(Volts.of(-0.7)));
    y.whileTrue(armSubsystem.runVoltsCommand(Volts.of(-0.7)));

    // Add Reset Pose Command
    SmartDashboard.putData("Reset Pose", Commands.runOnce(() -> {
      localizationSubsystem.resetPose(Pose2d.kZero);
    }, localizationSubsystem));

    try {
      SmartDashboard.putData("Command/Home", home());
      SmartDashboard.putData("Command/Score", scoreTeleop());
      SmartDashboard.putData("Command/Intake", intakeTeleop());
      SmartDashboard.putData("Command/L2", moveToLevel2());
      SmartDashboard.putData("Command/L3", moveToLevel3());
      SmartDashboard.putData("Command/L4", moveToLevel4());

      SmartDashboard.putData("ElevatorID/Volts0", elevatorSubsystem.runVoltsCommand(Volts.of(0.22)));
      SmartDashboard.putData("ElevatorID/Volts1", elevatorSubsystem.runVoltsCommand(Volts.of(0.24)));
      SmartDashboard.putData("ElevatorID/Volts2", elevatorSubsystem.runVoltsCommand(Volts.of(0.26)));
      SmartDashboard.putData("ElevatorID/Volts3", elevatorSubsystem.runVoltsCommand(Volts.of(1)));
      SmartDashboard.putData("ElevatorID/Volts4", elevatorSubsystem.runVoltsCommand(Volts.of(2)));

      SmartDashboard.putData("ElevatorSetpoints/0.0", elevatorSubsystem.goToPositionCommand(Meters.of(0.0)));
      SmartDashboard.putData("ElevatorSetpoints/0.2", elevatorSubsystem.goToPositionCommand(Meters.of(0.2)));
      SmartDashboard.putData("ElevatorSetpoints/0.4", elevatorSubsystem.goToPositionCommand(Meters.of(0.4)));
      SmartDashboard.putData("ElevatorSetpoints/0.6", elevatorSubsystem.goToPositionCommand(Meters.of(0.6)));

      SmartDashboard.putData("arm/Volts0", armSubsystem.runVoltsCommand(Volts.of(0.3)));
      SmartDashboard.putData("arm/Volts1", armSubsystem.runVoltsCommand(Volts.of(0.35)));
      SmartDashboard.putData("arm/Volts2", armSubsystem.runVoltsCommand(Volts.of(0.4)));
      SmartDashboard.putData("arm/Volts3", armSubsystem.runVoltsCommand(Volts.of(0.45)));
      SmartDashboard.putData("arm/Volts4", armSubsystem.runVoltsCommand(Volts.of(0.5)));

      SmartDashboard.putData("ArmSetpoints/-45", armSubsystem.goToPositionCommand(Degrees.of(-45)));
      SmartDashboard.putData("ArmSetpoints/0", armSubsystem.goToPositionCommand(Degrees.of(0)));
      SmartDashboard.putData("ArmSetpoints/45", armSubsystem.goToPositionCommand(Degrees.of(45)));

      SmartDashboard.putData("Align/A", POICommands.pathToCoralA());
      SmartDashboard.putData("Align/B", POICommands.pathToCoralB());
      SmartDashboard.putData("Align/C", POICommands.pathToCoralC());
      SmartDashboard.putData("Align/D", POICommands.pathToCoralD());
      SmartDashboard.putData("Align/E", POICommands.pathToCoralE());
      SmartDashboard.putData("Align/F", POICommands.pathToCoralF());
      SmartDashboard.putData("Align/G", POICommands.pathToCoralG());
      SmartDashboard.putData("Align/H", POICommands.pathToCoralH());
      SmartDashboard.putData("Align/I", POICommands.pathToCoralI());
      SmartDashboard.putData("Align/J", POICommands.pathToCoralJ());
      SmartDashboard.putData("Align/K", POICommands.pathToCoralK());
      SmartDashboard.putData("Align/L", POICommands.pathToCoralL());
      SmartDashboard.putData("Align/testPath", POICommands.pathToTestPath());
      SmartDashboard.putData("Align/IntakeLeft", POICommands.pathToLeftIntake());
      SmartDashboard.putData("Align/IntakeRight", POICommands.pathToRightIntake());

      SmartDashboard.putData("Elevator RE-HOME", elevatorCommands.reHome());

    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }
  }

  @SuppressWarnings("unused")
  private void elevatorTest() {
    a.whileTrue(elevatorSubsystem.goToPositionCommand(Meters.of(0.33)));
  }

  @SuppressWarnings("unused")
  private void armTest() {
    // b.whileTrue(arm.runOpenLoopCommand(Volts.of(2), Radians.of(1)));
    // x.whileTrue(arm.runOpenLoopCommand(Volts.of(-0.5), Radians.of(1.3)));
  }

  private void clawTest() {
  }

  @SuppressWarnings("unused")
  private void coralSubsystemTest() {
  }

  @SuppressWarnings("unused")
  private void sysIDSwerve() {
    a.whileTrue(swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    b.whileTrue(swerveSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    x.whileTrue(swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    y.whileTrue(swerveSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  @SuppressWarnings("unused")
  private void sysIDElevator() {
    a.whileTrue(elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    b.whileTrue(elevatorSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // x.whileTrue(elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // y.whileTrue(elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  @SuppressWarnings("unused")
  private void sysIDArm() {
    a.whileTrue(armSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    b.whileTrue(armSubsystem.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // x.whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // y.whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  private void autoAlignTest() throws FileVersionException, IOException, ParseException {
    buttonCoralA.whileTrue(POICommands.pathToCoralA());
    buttonCoralB.whileTrue(POICommands.pathToCoralB());
    buttonCoralC.whileTrue(POICommands.pathToCoralC());
    buttonCoralD.whileTrue(POICommands.pathToCoralD());
    buttonCoralE.whileTrue(POICommands.pathToCoralE());
    buttonCoralF.whileTrue(POICommands.pathToCoralF());
    buttonCoralG.whileTrue(POICommands.pathToCoralG());
    buttonCoralH.whileTrue(POICommands.pathToCoralH());
    buttonCoralI.whileTrue(POICommands.pathToCoralI());
    buttonCoralJ.whileTrue(POICommands.pathToCoralJ());
    buttonCoralK.whileTrue(POICommands.pathToCoralK());
    buttonCoralL.whileTrue(POICommands.pathToCoralL());

    buttonLeftIntake.whileTrue(algaeSubsystem.algaeShootCommand());
    buttonRightIntake.whileTrue(algaeSubsystem.undropCommand());
  }

  private void algaeTest() {
    rightBumper.whileTrue(algaeSubsystem.algaeIntakeCommand());
    leftBumper.whileTrue(algaeSubsystem.dropCommand());
  }

  private Command home() {
    Command command = Commands.parallel(
        armCommands.Home(),
        elevatorCommands.home());
    command.setName("Home");
    return command;
  }

  private Command moveToLevel2() {
    Command command = Commands.parallel(
        elevatorCommands.positionLevel2(),
        Commands.waitUntil(() -> elevatorSubsystem.getPosition().gt(Constants.armClearance))
            .andThen(armCommands.positionLevel2()));
    command.setName("Level 2");
    return command;
  }

  private Command moveToLevel3() {
    Command command = Commands.parallel(
        armCommands.positionLevel3(),
        elevatorCommands.positionLevel3());
    command.setName("Level 3");
    return command;
  }

  private Command moveToLevel4() {
    Command command = Commands.parallel(
        armCommands.positionLevel4(),
        elevatorCommands.positionLevel4());
    command.setName("Level 4");
    return command;
  }

  private Command intakeTeleop() {
    Command command = Commands.parallel(Commands.runOnce(() -> clawSubsystem.clearStallDetect(), clawSubsystem),
        armCommands.intakePosition(),
        elevatorCommands.intakeAction());

    command.setName("intake Coral teleop");
    return command;
  }

  // Intake auto does not return to the home position because this saves ~0.5
  // seconds
  private Command intakeAuto() {
    Command command = intakeTeleop();
    command.setName("intake Coral auto");
    return command;
  }

  // Scores just level 4
  @SuppressWarnings("unused")
  private Command scoreAuto() {
    Command command = armSubsystem.runVoltsCommand(Volts.of(-1.5)).withTimeout(0.4);
    command.setName("Score Auto");
    return command;
  }

  private Command scoreTeleop() {
    Command command = Commands.sequence(
        armCommands.score().withTimeout(Seconds.of(0.7)),
        Commands.race(
            armCommands.score(),
            swerveSubsystem.constantChassisSpeedsCommand(new ChassisSpeeds(-0.4, 0, 0)).withTimeout(Seconds.of(0.75))),
        armCommands.score());
    command.setName("Score");
    return command;
  }

  public Command lock() {
    Command command = swerveSubsystem.moduleLock(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-35)));
    return command;
  }

  private Command removeAlgae() {
    return armCommands.removeAlgae();
  }

}
