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
import frc.robot.Constants;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class SwerveDrive extends SubsystemBase {
  public SwerveModule[] modules;
  public SwerveModuleStates[] desiredModuleStates;
  public SwerveModuleLocation[] ModuleLocations;
  public SwerveDrivePoseEstimator poseEstimator;
  public ChassisSpeeds desiredChassisSpeeds;
  public SwerveModuleState desiredState;
  public Rotation2d desiredRotation;
  public double desiredX;
  public double desiredY;
  public XboxController controller;
  public ChassisSpeeds actualChassisSpeeds;
  public Translation2d stateVector;
  public SwerveDriveKinematics kinematics;
  public ADIS16470_IMU gyro = new ADIS16470_IMU();
  /** Creates a new SwerveDrive. */
  public SwerveDrive() {
    this.controller = new XboxController(0);
    gyro.update();
    desiredX = Math.cos(controller.getLeftX()) * Constants.SwerveConstants.bodyHeading;
    desiredY = Math.sin(controller.getLeftY()) * Constants.SwerveConstants.bodyHeading;
    desiredRotation = Rotation2d.fromDegrees(controller.getRightX() * 360);
    stateVector = Math.sqrt(Math.pow(desiredX,2) + Math.pow(desiredY,2));
    kinematics = new SwerveDriveKinematics(ModuleLocations);
    Translation2d[] ModuleLocation = new Translation2d[modules.length];
    poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroRotation(), getModulePositions(), new Pose2d(0,0,getGyroRotation())); // The pose estimator is odometry and provides positioning. It takes the kinematics, the module positions, the heading rotation, and the initial pose
    //actualChassisSpeeds = new ChassisSpeeds(desiredX,desiredY,controller.getRightX());
  }

  @Override
  public void periodic() {
    gyro.update();
    for (SwerveModule module : modules) {
      // Call periodic on all modules
      module.periodic();
  
    desiredState = new SwerveModuleState(stateVector, desiredRotation);
    poseEstimator.update(getGyroRotation(), getModulePositions());
    // This method will be called once per scheduler run
  }
  }
  public void getModulePositions() {
    for (int i = 0; i < modules.length; i++)
      ModuleLocations[i] = modules[i].getModulePosition();
    }
  public void getModuleStates() {
    setModuleStates(kinematic.toSwerveModuleStates(desiredChassisSpeeds););
  }
  public Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(gyro.getDegrees);
  }
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    for (int i = 0; i < desiredStates.length; i++) {
        modules[i].getDesiredState(desiredStates[i]);
    } //Assigns the desired states to their respective positions.
  }


}
