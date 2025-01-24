// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;

@Logged
public class Localization extends SubsystemBase {

  /// Loop time of the Localization update loop
  static Time fixedTimestep = Milliseconds.of(20);

  @NotLogged
  Swerve swerve;

  ADIS16470_IMU gyro = new ADIS16470_IMU();

  Rotation2d simulatorAngle = new Rotation2d();

  @NotLogged
  SwerveDriveOdometry odometry;

  @NotLogged
  SwerveDrivePoseEstimator poseEstimator;

  public Localization(Swerve swerve) {
    this.swerve = swerve;

    odometry = new SwerveDriveOdometry(
      SwerveConstants.kinematics,
        getGyroAngle(),
        swerve.getModulePositions());

    poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.kinematics, getGyroAngle(),
        swerve.getModulePositions(), Pose2d.kZero);
  }


  /// THIS IS THE RAW GYRO ANGLE NOT THE ESTIMATED ROBOT ANGLE
  private Rotation2d getGyroAngle() {
    return Rotation2d.fromDegrees(gyro.getAngle());
  }


  public Pose2d getRobotPose() {
    return poseEstimator.getEstimatedPosition();
  }

  @Override
  public void periodic() {
    if (Robot.isSimulation()) {
      // Only run simulation periodic on simulated robots
      return;
    }

    odometry.update(getGyroAngle(), swerve.getModulePositions());
    poseEstimator.update(getGyroAngle(), swerve.getModulePositions());

    // TODO add vision measurements
  }

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    /// Estimate the angle of the robot purely from the wheel encoders, this is good
    /// enough for a simulated robot.
    ChassisSpeeds chassisSpeedsFromOdometry = SwerveConstants.kinematics.toChassisSpeeds(swerve.getModuleStates());
    simulatorAngle = simulatorAngle
        .plus(Rotation2d.fromRadians(chassisSpeedsFromOdometry.omegaRadiansPerSecond * fixedTimestep.in(Seconds)));

    odometry.update(simulatorAngle, swerve.getModulePositions());
    poseEstimator.update(simulatorAngle, swerve.getModulePositions());
  }
}
