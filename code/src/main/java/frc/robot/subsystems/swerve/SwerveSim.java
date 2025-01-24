// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;


import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class SwerveSim extends SubsystemBase implements SwerveBase {

  SwerveModuleSim fl = new SwerveModuleSim();
  SwerveModuleSim fr = new SwerveModuleSim();
  SwerveModuleSim bl = new SwerveModuleSim();
  SwerveModuleSim br = new SwerveModuleSim();

  @NotLogged
  SwerveModuleSim[] modules = {
    fl,
    fr,
    bl,
    br,
  };

  static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.moduleLocations);

  /** Creates a new SwerveSim. */
  public SwerveSim() {
  }

  @Override
  public void periodic() {

    setChassisSpeeds(new ChassisSpeeds(1, 0, 3.14));

    // This method will be called once per scheduler run
    for(SwerveModuleSim module : modules) {
      module.periodic();
    }
  }

  @Override
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] modulePositions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            modulePositions[i] = modules[i].getPosition();
        }
        return modulePositions;
  }

  @Override
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] moduleStates = new SwerveModuleState[modules.length];
    for (int i = 0; i < modules.length; i++) {
        moduleStates[i] = modules[i].getState();
    }
    return moduleStates;
  }

  @Override
  public void setChassisSpeeds(ChassisSpeeds speeds) {
    setModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  @Override
  public void stop() {
    setChassisSpeeds(new ChassisSpeeds(0, 0, 0));
  }

  @Override @NotLogged
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  @Override
  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      modules[i].setState(states[i]);
    }
  }
}
