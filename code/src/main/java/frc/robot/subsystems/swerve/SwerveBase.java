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

@Logged
public interface SwerveBase {

  @NotLogged
  public SwerveDriveKinematics getKinematics ();

  public SwerveModulePosition[] getModulePositions ();

  public SwerveModuleState[] getModuleStates ();

  public void setModuleStates (SwerveModuleState[] states);

  public void setChassisSpeeds (ChassisSpeeds speeds);

  public void stop ();
}
