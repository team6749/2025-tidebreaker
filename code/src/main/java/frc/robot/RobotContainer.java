// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Radians;
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
import frc.robot.subsystems.IntakeDropper;

@Logged
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  ActiveClawSubsystem clawSubsystem;
  ClimberSubsystem climberSubsystem;
  SwerveDrive swerveSubsystem;
  ConstrainedArmSubsystem arm;
  AlgaeSubsystem algaeSubsystem;
  Localization localizationSubsystem;
  Elevator elevatorSubsystem;
  ElevatorCommands elevatorCommands;
  ArmCommands armCommands;
  POICommands poiCommands;
  IntakeDropper intakeDropper;

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

  JoystickButton buttonLevel2 = new JoystickButton(bottomButtonBoard, 1);
  JoystickButton buttonLevel3 = new JoystickButton(bottomButtonBoard, 2);
  JoystickButton buttonLevel4 = new JoystickButton(bottomButtonBoard, 3);
  JoystickButton buttonHome = new JoystickButton(bottomButtonBoard, 4);
  JoystickButton buttonScore = new JoystickButton(bottomButtonBoard, 5);
  JoystickButton buttonIntake = new JoystickButton(bottomButtonBoard, 6);
  JoystickButton buttonTopAlgae = new JoystickButton(bottomButtonBoard, 7);
  JoystickButton buttonBottomAlgae = new JoystickButton(bottomButtonBoard, 8);
  JoystickButton buttonCoralC = new JoystickButton(bottomButtonBoard, 9);
  JoystickButton buttonCoralD = new JoystickButton(bottomButtonBoard, 10);
  JoystickButton buttonCoralF = new JoystickButton(bottomButtonBoard, 11);
  JoystickButton buttonCoralE = new JoystickButton(bottomButtonBoard, 12);

  JoystickButton buttonCoralL = new JoystickButton(topButtonBoard, 1);
  JoystickButton buttonCoralK = new JoystickButton(topButtonBoard, 2);
  JoystickButton buttonCoralJ = new JoystickButton(topButtonBoard, 3);
  JoystickButton buttonCoralI = new JoystickButton(topButtonBoard, 4);
  JoystickButton buttonCoralB = new JoystickButton(topButtonBoard, 5);
  JoystickButton buttonIntakeDrop = new JoystickButton(topButtonBoard, 6);
  JoystickButton buttonCoralH = new JoystickButton(topButtonBoard, 7);
  JoystickButton buttonCoralA = new JoystickButton(topButtonBoard, 8);
  JoystickButton buttonCage = new JoystickButton(topButtonBoard, 9);
  JoystickButton buttonRightIntake = new JoystickButton(topButtonBoard, 10);
  JoystickButton buttonLeftIntake = new JoystickButton(topButtonBoard, 11);
  JoystickButton buttonCoralG = new JoystickButton(topButtonBoard, 12);

  public RobotContainer() {
    swerveSubsystem = new SwerveDrive();
    arm = new ConstrainedArmSubsystem();
    clawSubsystem = new ActiveClawSubsystem();
    algaeSubsystem = new AlgaeSubsystem();
    localizationSubsystem = new Localization(swerveSubsystem);
    elevatorSubsystem = new Elevator();
    poiCommands = new POICommands(swerveSubsystem);
    elevatorCommands = new ElevatorCommands(elevatorSubsystem);
    armCommands = new ArmCommands(arm);
    climberSubsystem = new ClimberSubsystem();
    intakeDropper = new IntakeDropper();

    NamedCommands.registerCommand("home", home());
    NamedCommands.registerCommand("intake", intakeAuto());
    NamedCommands.registerCommand("wait_for_coral", Commands.idle(elevatorSubsystem)
        .until(() -> elevatorSubsystem.getIsCoralLimitSwitchActivated()).withTimeout(Robot.isSimulation() ? 1.25 : 2.5));
    NamedCommands.registerCommand("score_l4", clawSubsystem.clawHighShoot().withTimeout(Seconds.of(0.5)));
    NamedCommands.registerCommand("l2", moveToLevel2());
    NamedCommands.registerCommand("l3", moveToLevel3());
    NamedCommands.registerCommand("l4", moveToLevel4());

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

    // Add Rest Pose Command
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

      SmartDashboard.putData("arm/Volts0", arm.runVoltsCommand(Volts.of(0.3)));
      SmartDashboard.putData("arm/Volts1", arm.runVoltsCommand(Volts.of(0.35)));
      SmartDashboard.putData("arm/Volts2", arm.runVoltsCommand(Volts.of(0.4)));
      SmartDashboard.putData("arm/Volts3", arm.runVoltsCommand(Volts.of(0.45)));
      SmartDashboard.putData("arm/Volts4", arm.runVoltsCommand(Volts.of(0.5)));

      SmartDashboard.putData("ArmSetpoints/-45", arm.goToPositionCommand(Degrees.of(-45)));
      SmartDashboard.putData("ArmSetpoints/0", arm.goToPositionCommand(Degrees.of(0)));
      SmartDashboard.putData("ArmSetpoints/45", arm.goToPositionCommand(Degrees.of(45)));

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
      SmartDashboard.putData("Align/testPath", poiCommands.pathToTestPath());
      SmartDashboard.putData("Align/IntakeLeft", poiCommands.pathToLeftIntake());
      SmartDashboard.putData("Align/IntakeRight", poiCommands.pathToRightIntake());

      SmartDashboard.putData("Intake/Drop", intakeDropper.drop());
      SmartDashboard.putData("Intake/Hold", intakeDropper.hold());

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
    a.whileTrue(arm.goToPositionCommand(Radians.of(0)));
    // b.whileTrue(arm.runOpenLoopCommand(Volts.of(2), Radians.of(1)));
    // x.whileTrue(arm.runOpenLoopCommand(Volts.of(-0.5), Radians.of(1.3)));
  }
  private void clawTest() {
    b.whileTrue(clawSubsystem.clawLowShoot());
    x.whileTrue(clawSubsystem.clawHighShoot());
  }

  @SuppressWarnings("unused")
  private void coralSubsystemTest() {
    buttonHome.whileTrue(home());
    buttonLevel2.whileTrue(moveToLevel2());
    buttonLevel3.whileTrue(moveToLevel3());
    buttonLevel4.whileTrue(moveToLevel4().onlyIf(() -> elevatorSubsystem.getIsCoralLimitSwitchActivated() == false));
    buttonIntake.whileTrue(intakeTeleop());
    buttonScore.whileTrue(scoreTeleop());
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
    // x.whileTrue(elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // y.whileTrue(elevatorSubsystem.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  private void sysIDArm() {
    a.whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    b.whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    // x.whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kForward));
    // y.whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kReverse));
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
    buttonLeftIntake.whileTrue(algaeSubsystem.algaeShootCommand());
  }

  private void algaeTest() {
    b.whileTrue(algaeSubsystem.algaeIntakeCommand());
    a.whileTrue(algaeSubsystem.algaeShootCommand());
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
    Command command = Commands.sequence(
        elevatorCommands.home(),
        armCommands.positionLevel3(),
        elevatorCommands.positionLevel3());
    command.setName("Level 3");
    return command;
  }

  private Command moveToLevel4() {
    Command command = Commands.parallel(
        elevatorCommands.positionLevel4(),
        Commands.waitUntil(() -> elevatorSubsystem.getPosition().gt(Constants.armClearance))
            .andThen(armCommands.positionLevel4()));
    command.setName("Level 4");
    return command;
  }

  private Command intakeTeleop() {
    Command command = Commands.parallel(
      armCommands.intakePosition(),
      elevatorCommands.intakeAction()
    );
        
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
  private Command scoreAuto() {
    Command command = arm.runVoltsCommand(Volts.of(-1.5)).withTimeout(0.4);
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
    Command command = swerveSubsystem.moduleLock(new SwerveModuleState (0.0,Rotation2d.fromDegrees(-35)));
    return command;
  }

}
