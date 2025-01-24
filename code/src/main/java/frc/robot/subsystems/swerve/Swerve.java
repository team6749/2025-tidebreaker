// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;


import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Swerve extends SubsystemBase {

  SwerveModuleSim frontLeft;
  SwerveModuleSim frontRight;
  SwerveModuleSim backLeft;
  SwerveModuleSim backRight;

  @NotLogged
  SwerveModuleSim[] modules = new SwerveModuleSim[4];

  /** Creates a new SwerveSim. */
  public Swerve() {
    
    if(RobotBase.isSimulation()) {
      frontLeft = new SwerveModuleSim();
      frontRight = new SwerveModuleSim();
      backLeft = new SwerveModuleSim();
      backRight = new SwerveModuleSim();
    } else {
      // TODO Robot is real, so use real swerve module definitions
    }

    modules[0] = frontLeft;
    modules[1] = frontRight;
    modules[2] = backLeft;
    modules[3] = backRight;

  }

  @Override
  public void periodic() {

    setChassisSpeeds(new ChassisSpeeds(1, 0, 3.14));

    // This method will be called once per scheduler run
    for(SwerveModuleSim module : modules) {
      module.periodic();
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

  public void setChassisSpeeds(ChassisSpeeds speeds) {
    setModuleStates(SwerveConstants.kinematics.toSwerveModuleStates(speeds));
  }

  public void stop() {
    for(SwerveModuleSim module : modules) {
      module.stop();
    }
  }

  public void setModuleStates(SwerveModuleState[] states) {
    for (int i = 0; i < states.length; i++) {
      modules[i].setState(states[i]);
    }
  }
}
