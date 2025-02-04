// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveConstants;

@Logged
public class Localization extends SubsystemBase {

    Alert gyroDisconnectAlert = new Alert("Gyro disconnected, falling back to kinematics", AlertType.kError);

    @NotLogged
    SwerveDrive swerve;

    // Used to infer chassis rotation from swerve module states. this drifts quickly
    Angle chassisRotation = Radians.zero();

    boolean isGyroConnected = true;
    ADIS16470_IMU gyro = new ADIS16470_IMU();

    @NotLogged
    SwerveDriveOdometry odometry;

    @NotLogged
    SwerveDrivePoseEstimator poseEstimator;

    // Simulation
    ADIS16470_IMUSim gyroSim = new ADIS16470_IMUSim(gyro);

    Field2d dashboardField = new Field2d();

    public Localization(SwerveDrive swerve) {
        this.swerve = swerve;

        odometry = new SwerveDriveOdometry(
                SwerveConstants.kinematics,
                getGyroAngle(),
                swerve.getModulePositions());

        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.kinematics, getGyroAngle(),
                swerve.getModulePositions(), Pose2d.kZero);

        SmartDashboard.putData("Field", dashboardField);
        gyro.calibrate();
    }

    /// THIS IS THE RAW GYRO ANGLE NOT THE ESTIMATED ROBOT ANGLE
    private Rotation2d getGyroAngle() {
        if(isGyroConnected == false) {
            return Rotation2d.fromRadians(chassisRotation.in(Radians));
        }
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }

    @Override
    public void periodic() {
        chassisRotation = chassisRotation.plus(Radians.of(Constants.simulationTimestep.in(Seconds) * SwerveConstants.kinematics.toChassisSpeeds(swerve.getModuleStates()).omegaRadiansPerSecond));
        isGyroConnected = gyro.isConnected();
        gyroDisconnectAlert.set(isGyroConnected == false);
        if(isGyroConnected) {
            chassisRotation = getGyroAngle().getMeasure();
        }

        odometry.update(getGyroAngle(), swerve.getModulePositions());
        poseEstimator.update(getGyroAngle(), swerve.getModulePositions());

        // TODO add vision measurements

        // Update Dashboard (this is for elastic/driver)
        dashboardField.setRobotPose(getRobotPose());
    }

    @Override
    public void simulationPeriodic() {
        super.simulationPeriodic();

        /// Estimate the angle of the robot purely from the wheel encoders, this is good
        /// enough for a simulated robot.
        ChassisSpeeds chassisSpeedsFromKinematics = SwerveConstants.kinematics
                .toChassisSpeeds(swerve.getModuleStates());

        // Update the gyro with the simulated data
        gyroSim.setGyroRateZ(Rotation2d.fromRadians(chassisSpeedsFromKinematics.omegaRadiansPerSecond).getDegrees());
        gyroSim.setGyroAngleZ(gyro.getAngle() + (gyro.getRate() * Constants.simulationTimestep.in(Seconds)));

    }

}
