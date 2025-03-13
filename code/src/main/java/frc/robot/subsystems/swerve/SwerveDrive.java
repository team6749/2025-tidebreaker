// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.enums.DriveOrientation;
import frc.robot.subsystems.Localization;

@Logged
public class SwerveDrive extends SubsystemBase {
    private final SendableChooser<DriveOrientation> orientationChooser = new SendableChooser<>();
    SwerveModuleBase frontLeft;
    SwerveModuleBase frontRight;
    SwerveModuleBase backLeft;
    SwerveModuleBase backRight;

    @NotLogged
    SwerveModuleBase[] modules = new SwerveModuleBase[4];

    // For logging purposes right now.
    ChassisSpeeds loggedTargetChassisSpeeds = new ChassisSpeeds();

    public SwerveDrive() {
        if (RobotBase.isSimulation()) {
            frontLeft = new SwerveModuleSim();
            frontRight = new SwerveModuleSim();
            backLeft = new SwerveModuleSim();
            backRight = new SwerveModuleSim();

        } else {
            frontLeft = new SwerveModuleReal(SwerveConstants.FLDriveMotorPort, SwerveConstants.FLAngleMotorPort,
                    SwerveConstants.FLEncoderPort);
            frontRight = new SwerveModuleReal(SwerveConstants.FRDriveMotorPort, SwerveConstants.FRAngleMotorPort,
                    SwerveConstants.FREncoderPort);
            backLeft = new SwerveModuleReal(SwerveConstants.BLDriveMotorPort, SwerveConstants.BLAngleMotorPort,
                    SwerveConstants.BLEncoderPort);
            backRight = new SwerveModuleReal(SwerveConstants.BRDriveMotorPort, SwerveConstants.BRAngleMotorPort,
                    SwerveConstants.BREncoderPort);
        }

        modules[0] = frontLeft;
        modules[1] = frontRight;
        modules[2] = backLeft;
        modules[3] = backRight;

        orientationChooser.setDefaultOption("Field Orientation", DriveOrientation.FieldOriented);
        orientationChooser.addOption("Robot Orientation", DriveOrientation.RobotOriented);
        SmartDashboard.putData("Robot Orientation Mode", orientationChooser);
    }

    @Override
    public void periodic() {
        if (RobotState.isDisabled()) {
            stop();
        }

        for (int i = 0; i < modules.length; i++) {
            modules[i].periodic();
        }
    }

    public double inputMagnitude(double x, double y) {
        return Math.sqrt((x * x) + (y * y));
    };

    public double exponentialResponseCurve(double input) {
        return Math.pow(input, 5);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            modulePositions[i] = modules[i].getPosition();
        }
        return modulePositions;
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            moduleStates[i] = modules[i].getState();
        }
        return moduleStates;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return SwerveConstants.kinematics.toChassisSpeeds(getModuleStates());

    }

    /// Chassis Speeds can saturate the modules, this method desaturates the modules
    public void runChassisSpeeds(ChassisSpeeds speeds) {
        loggedTargetChassisSpeeds = speeds;
        runModuleStates(SwerveConstants.kinematics.toSwerveModuleStates(speeds));
    }

    public void runModuleStates(SwerveModuleState[] states) {
        loggedTargetChassisSpeeds = SwerveConstants.kinematics.toChassisSpeeds(states);
        for (int i = 0; i < states.length; i++) {
            modules[i].runClosedLoop(states[i]);
        }
    }

    public void stop() {
        for (SwerveModuleBase module : modules) {
            module.stop();
        }
    }

    public Command constantChassisSpeedsCommand(ChassisSpeeds speeds) {
        Command command = Commands.runEnd(() -> {
            runChassisSpeeds(speeds);
        }, () -> {
            stop();
        }, this);
        command.setName("Constant Chasssis Speeds Command");
        return command;
    }

    // Simple robot relative drive with no field oriented control, response curves,
    // deadbands, or slew rates
    public Command basicDriveCommand(XboxController controller, Localization localizationSubsystem) {

        Command command = Commands.runEnd(() -> {

            double xInput = -controller.getLeftY();
            double yInput = -controller.getLeftX();
            double zInput = -controller.getRightX();

            if (inputMagnitude(xInput, yInput) < Constants.deadZone) {
                xInput = 0;
                yInput = 0;
            }
            if (inputMagnitude(zInput, 0) < Constants.deadZone) {
                zInput = 0;
            }

            ChassisSpeeds targetSpeeds = new ChassisSpeeds(

                    MetersPerSecond.of(SwerveConstants.driveLimiterY.calculate(SwerveConstants.maxLinearVelocity
                            .times(exponentialResponseCurve(xInput)).in(MetersPerSecond))),
                    MetersPerSecond.of(SwerveConstants.driveLimiterX.calculate(SwerveConstants.maxLinearVelocity
                            .times(exponentialResponseCurve(yInput)).in(MetersPerSecond))),
                    RadiansPerSecond.of(SwerveConstants.driveLimiterTheta.calculate(SwerveConstants.maxAngularVelocity
                            .times(exponentialResponseCurve(zInput)).in(RadiansPerSecond))));

            switch (orientationChooser.getSelected()) {
                case FieldOriented:
                    Rotation2d robotRotation2d = localizationSubsystem.getRobotPose().getRotation();
                    targetSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(targetSpeeds,
                            DriverStation.getAlliance().get() == Alliance.Blue ? robotRotation2d
                                    : robotRotation2d.plus(Rotation2d.fromDegrees(180)));

                    break;
                default:
                    break;
            }

            // Desaturate the input
            SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(targetSpeeds);
            SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.maxLinearVelocity);
            runModuleStates(states);

        }, () -> {
            stop();
        }, this);
        command.setName("Base Drive Command");
        return command;
    }

    public Command testModuleSpeeds(SwerveModuleState target) {
        Command command = Commands.runEnd(() -> {
            // targetSpeeds = new ChassisSpeeds(1,0,0);
            // targetSpeeds = new ChassisSpeeds(3,0,0);
            // targetSpeeds = new ChassisSpeeds(0,0.5,0);
            // targetSpeeds = new ChassisSpeeds(0,0,1);
            runModuleStates(new SwerveModuleState[] { target, target, target, target });
        }, () -> {
            stop();
        }, this);
        return command;
    }
}
