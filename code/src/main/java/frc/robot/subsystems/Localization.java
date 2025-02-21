// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.ADIS16470_IMUSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.subsystems.swerve.SwerveConstants;

@Logged
public class Localization extends SubsystemBase {

    Alert frontLimelightFailure = new Alert("Front Limelight Failure", AlertType.kError);
    Alert backLimelightFailure = new Alert("Back Limelight Failure", AlertType.kError);
    boolean applyLimePositioning = false; // For now, log only, don't actually apply to odometry

    @NotLogged
    SwerveDrive swerve;

    ADIS16470_IMU gyro = new ADIS16470_IMU();
    public static final String LimeLightFront = "limelight-front";
    public static final String LimeLightBack = "limelight-back";

    @NotLogged
    SwerveDriveOdometry odometry;

    Pose2d backLimelight = null;
    Pose2d frontLimelight = null;

    @NotLogged
    SwerveDrivePoseEstimator poseEstimator;

    // Simulation
    ADIS16470_IMUSim gyroSim = new ADIS16470_IMUSim(gyro);

    Field2d dashboardField = new Field2d();

    private final SendableChooser<Boolean> limelightToggleChooser = new SendableChooser<>();

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

        limelightToggleChooser.setDefaultOption("Disabled", false);
        limelightToggleChooser.addOption("Enabled", true);
        SmartDashboard.putData("Limelight Toggle", limelightToggleChooser);
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

        // allow the smartdashboard to toggle vision updates
        applyLimePositioning = limelightToggleChooser.getSelected();

        odometry.update(getGyroAngle(), swerve.getModulePositions());
        poseEstimator.update(getGyroAngle(), swerve.getModulePositions());

        LimelightHelpers.SetRobotOrientation(LimeLightFront,
                poseEstimator.getEstimatedPosition().getRotation().getRadians(), 0, 0, 0, 0, 0);
        PoseEstimate mt1Front = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimeLightFront);
        frontLimelightFailure.set(mt1Front == null);
        if (mt1Front != null) {
            frontLimelight = applyVisionUpdates(mt1Front);
        }

        LimelightHelpers.SetRobotOrientation(LimeLightBack,
                poseEstimator.getEstimatedPosition().getRotation().getRadians(), 0, 0, 0, 0, 0);
        PoseEstimate mt1Back = LimelightHelpers.getBotPoseEstimate_wpiBlue(LimeLightBack);
        backLimelightFailure.set(mt1Back == null);
        if (mt1Back != null) {
            backLimelight = applyVisionUpdates(mt1Back);
        }

        // Update Dashboard (this is for elastic/driver)
        dashboardField.setRobotPose(getRobotPose());
    }

    private Pose2d applyVisionUpdates(LimelightHelpers.PoseEstimate mt1) {
        if (mt1.tagCount > 0 && applyLimePositioning) {

            Distance dist = Meters.of(9999);
            double ambiguity = 1;
            if (mt1.rawFiducials.length > 0) {
                dist = Meters.of(mt1.rawFiducials[0].distToCamera);
                ambiguity = mt1.rawFiducials[0].ambiguity;
            }

            double xSpeedMetersPerSecond = swerve.getChassisSpeeds().vxMetersPerSecond;
            double ySpeedMetersPerSecond = swerve.getChassisSpeeds().vyMetersPerSecond;

            var trust = calculateMeasurementTrust(
                    MetersPerSecond.of(Math.hypot(xSpeedMetersPerSecond, ySpeedMetersPerSecond)),
                    DegreesPerSecond.of(gyro.getRate()), dist, ambiguity);

            poseEstimator.setVisionMeasurementStdDevs(trust);
            poseEstimator.addVisionMeasurement(
                    mt1.pose,
                    mt1.timestampSeconds);
        }
        return mt1.pose;
    }

    // ambiguity is a limelight specific term, with values between 0 and 1, and 1 is
    // the "worst", anything greater than 0.7 is "bad"
    Matrix<N3, N1> calculateMeasurementTrust(LinearVelocity robotSpeedMetersPerSecond,
            AngularVelocity robotAnglePerSecond,
            Distance tagDistance, double ambiguity) {
        final Matrix<N3, N1> rejectValue = VecBuilder.fill(9999999, 9999999, 9999999);
        final Matrix<N3, N1> acceptValue = VecBuilder.fill(8, 8, 128);
        final Matrix<N3, N1> idealValue = VecBuilder.fill(0.5, 0.5, 24);

        var returnValue = idealValue;

        // Not Ideal but still accept Conditions
        if (robotSpeedMetersPerSecond.gt(MetersPerSecond.of(0.3))) {
            // reject when moving quickly
            returnValue = acceptValue;
        }
        if (robotAnglePerSecond.in(DegreesPerSecond) > 5) {
            // reject when rotating
            returnValue = acceptValue;
        }
        if (tagDistance.in(Meters) > 1.5) {
            // reject far away tags
            returnValue = acceptValue;
        }

        // Reject Conditions
        if (ambiguity > 0.7) {
            returnValue = rejectValue;
        }
        if (robotSpeedMetersPerSecond.gt(MetersPerSecond.of(2))) {
            // reject when moving quickly
            returnValue = rejectValue;
        }
        if (robotAnglePerSecond.in(DegreesPerSecond) > 30) {
            // reject when rotating
            returnValue = rejectValue;
        }
        if (tagDistance.in(Meters) > 3) {
            // reject far away tags
            returnValue = rejectValue;
        }

        // Speed range: 0 - 3 m/s
        // Rotation Range: 0 - 60 deg/s
        // Distance: 0 - 3 m
        // ambugity: 0 - 0.7

        return returnValue;
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