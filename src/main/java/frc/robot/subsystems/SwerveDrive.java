// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {
  public SwerveModule[] modulePositions[];
  public SwerveDrivePoseEstimator poseEstimator;
  public ChassisSpeeds desiredChassisSpeeds;
  public SwerveModuleState desiredState;
  public Rotation2d desiredRotation;
  public double desiredX;
  public double desiredY;
  public XboxController controller;
  public ChassisSpeeds actualChassisSpeeds;
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    this.controller = new XboxController(0);
    desiredX = Math.cos(controller.getLeftX());
    desiredY = Math.sin(controller.getLeftY());
    desiredRotation = Rotation2d.fromDegrees(controller.getRightX() * 360);
    actualChassisSpeeds = new ChassisSpeeds(desiredX,desiredY,controller.getRightX());
  }

  @Override
  public void periodic() {
    desiredState = new SwerveModuleState(Math.sqrt(Math.pow(desiredX,2) + Math.pow(desiredY,2)), desiredRotation);
    // This method will be called once per scheduler run
  }

}
