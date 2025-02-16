// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;

import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
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

/**
 * The Localization subsystem tracks the robot's position on the field using odometry 
 * and sensor data.
 * 
 * <p>This class maintains both {@link SwerveDriveOdometry} and 
 * {@link SwerveDrivePoseEstimator} to estimate the robot’s pose using swerve 
 * module states and gyro data. It supports simulation and integrates with 
 * {@link PathPlannerLogging} for visualization.
 * 
 * <p>Features:
 * <ul>
 *   <li>Tracks robot pose using swerve odometry and pose estimation</li>
 *   <li>Uses an ADIS16470 IMU for gyro-based heading estimation</li>
 *   <li>Supports vision integration for future enhancements</li>
 *   <li>Provides real-time pose updates to SmartDashboard</li>
 *   <li>Simulates robot movement in a virtual field environment</li>
 * </ul>
 * 
 * This subsystem ensures accurate positioning data for autonomous navigation 
 * and field-oriented control.
 */
@Logged
public class Localization extends SubsystemBase {

    Alert frontLimelightFailure = new Alert("Front Limelight Failure", AlertType.kError);
    Alert backLimelightFailure = new Alert("Back Limelight Failure", AlertType.kError);
    boolean applyLimePositioning = false; //For now, log only, don't actually apply to odometry

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

        LimelightHelpers.SetRobotOrientation(LimeLightFront, poseEstimator.getEstimatedPosition().getRotation().getRadians(), 0, 0, 0, 0, 0);
        PoseEstimate mt2Front = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimeLightFront);
        frontLimelightFailure.set(mt2Front == null);
        frontLimelight = applyVisionUpdates(mt2Front);

        LimelightHelpers.SetRobotOrientation(LimeLightBack, poseEstimator.getEstimatedPosition().getRotation().getRadians(), 0, 0, 0, 0, 0);
        PoseEstimate mt2Back = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LimeLightBack);
        backLimelightFailure.set(mt2Back == null);
        backLimelight = applyVisionUpdates(mt2Back);

        // Update Dashboard (this is for elastic/driver)
        dashboardField.setRobotPose(getRobotPose());
    }


    private Pose2d applyVisionUpdates(LimelightHelpers.PoseEstimate mt2) {
        boolean doRejectUpdate = false;
        if (mt2 == null) {
            return null;
        }
        if(mt2.tagCount == 0 || Math.abs(gyro.getRate()) > 720) {
             // if our angular velocity is greater than 720 degrees per second, ignore vision updates
            // OR We don't see any tags currently
          doRejectUpdate = true;
        }

        if(!doRejectUpdate && applyLimePositioning)
        {
          poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
          poseEstimator.addVisionMeasurement(
              mt2.pose,
              mt2.timestampSeconds);
        }
        return mt2.pose;
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