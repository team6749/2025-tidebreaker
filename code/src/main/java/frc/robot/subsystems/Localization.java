// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveConstants;

@Logged
public class Localization extends SubsystemBase {

    @NotLogged
    SwerveDrive swerve;

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

                // Logging callback for target robot pose
        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            // Do whatever you want with the pose here
            dashboardField.getObject("target pose").setPose(pose);
        });

        // Logging callback for the active path, this is sent as a list of poses
        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            // Do whatever you want with the poses here
            dashboardField.getObject("path").setPoses(poses);
        });

        SmartDashboard.putData("Field", dashboardField);
    }

    /// THIS IS THE RAW GYRO ANGLE NOT THE ESTIMATED ROBOT ANGLE
    private Rotation2d getGyroAngle() {
        return Rotation2d.fromDegrees(gyro.getAngle());
    }

    public Pose2d getRobotPose() {
        return poseEstimator.getEstimatedPosition();
    }
    public void resetPose(Pose2d newPose) {
        odometry.resetPose(newPose);
        poseEstimator.resetPose(newPose);
    }

    @Override
    public void periodic() {

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