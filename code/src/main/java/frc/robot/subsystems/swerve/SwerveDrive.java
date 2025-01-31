// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Localization;

@Logged
public class SwerveDrive extends SubsystemBase {

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
           frontLeft = new SwerveModuleReal(getName(), SwerveConstants.FLDriveMotorPort, SwerveConstants.FLAngleMotorPort, SwerveConstants.FLEncoderPort);
           frontRight = new SwerveModuleReal(getName(), SwerveConstants.FRDriveMotorPort, SwerveConstants.FRAngleMotorPort, SwerveConstants.FREncoderPort);
           backLeft = new SwerveModuleReal(getName(),SwerveConstants.BLDriveMotorPort, SwerveConstants.BLAngleMotorPort, SwerveConstants.BLEncoderPort);
           backRight = new SwerveModuleReal(getName(), SwerveConstants.BRDriveMotorPort, SwerveConstants.BRAngleMotorPort, SwerveConstants.BREncoderPort);
        }

        modules[0] = frontLeft;
        modules[1] = frontRight;
        modules[2] = backLeft;
        modules[3] = backRight;
    }

    @Override
    public void periodic() {
        if (RobotState.isDisabled()) {
            stop();
        }

        for(int i = 0; i < modules.length; i++) {
            modules[i].periodic();
        }
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
    public Command basicDriveCommand(XboxController controller) {
        Command command = Commands.runEnd(() -> {
            ChassisSpeeds targetSpeeds = new ChassisSpeeds(
                    SwerveConstants.maxLinearVelocity.times(-controller.getLeftX()),
                    SwerveConstants.maxLinearVelocity.times(-controller.getLeftY()),
                    SwerveConstants.maxAngularVelocity.times(-controller.getRightX()));

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
}
