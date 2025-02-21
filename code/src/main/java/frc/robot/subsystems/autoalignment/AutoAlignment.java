package frc.robot.subsystems.autoalignment;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.Input;
import frc.robot.subsystems.Localization;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.*;

public class AutoAlignment extends SubsystemBase {
    private final Localization localization;
    private AutoAlignmentLoader autoAlignmentLoader;
    private int updateCounter = 0; // Counter for 500ms intervals
    private Translation2d targetLocation;
    private Rotation2d targetRotation;
    private final double[] lastValues = new double[]{Double.MAX_VALUE, Double.MAX_VALUE};

    public AutoAlignment(Localization localization) {
        this.localization = localization;
        this.autoAlignmentLoader = new AutoAlignmentLoader();
    }

    @Override
    public void periodic() {
        // Update UI every 500ms (25 loops at 20ms per loop)
        if (updateCounter++ % 25 == 0) {
            autoAlignmentLoader.update();
        }
    }

    public String findNearestReefTarget() {
        Pose2d robotPose = localization.getRobotPose();
        Translation2d robotPosition = robotPose.getTranslation();

        return autoAlignmentLoader.getCoralSproutPositions().entrySet().stream()
            .min(Comparator.comparingDouble(reef -> reef.getValue().getTranslation().getDistance(robotPosition)))
            .map(Map.Entry::getKey)
            .orElse(null);
    }

    public Pose2d findNearestReefPositionBySproutName(String sproutName) {
        return autoAlignmentLoader.getCoralSproutPositions().get(sproutName);
    }

    public Command autoAlignCommand(SwerveDrive swerveDrive, Input inputSubsystem) {
        return new FunctionalCommand(
            () -> {
                String coralSproutName = findNearestReefTarget();
                Pose2d coralSproutPose = findNearestReefPositionBySproutName(coralSproutName);
                targetLocation = coralSproutPose != null ? coralSproutPose.getTranslation() : null;
                targetRotation = coralSproutPose != null ? coralSproutPose.getRotation() : null;
                lastValues[0] = Double.MAX_VALUE; // lastDistance
                lastValues[1] = Double.MAX_VALUE; // lastRotationError
            },
            () -> {
                if (targetLocation != null) {
                    Pose2d robotPose = localization.getRobotPose();
                    Translation2d robotPosition = robotPose.getTranslation();
                    Rotation2d robotRotation = robotPose.getRotation();

                    Translation2d movementVector = targetLocation.minus(robotPosition);
                    double distanceToTarget = movementVector.getNorm();
                    double rotationError = targetRotation.minus(robotRotation).getRadians();

                    if (distanceToTarget > lastValues[0]) {
                        swerveDrive.stop();
                        return;
                    }
                    lastValues[0] = distanceToTarget;

                    Translation2d movementDirection = movementVector.div(distanceToTarget);
                    double speedFactor = Math.min(distanceToTarget * AutoAlignmentConstants.DECELERATION_FACTOR, 
                                                AutoAlignmentConstants.DECELERATION_LIMIT);

                    double forwardSpeed = movementDirection.getX() * speedFactor;
                    double strafeSpeed = movementDirection.getY() * speedFactor;

                    double rotationSpeed = rotationError * AutoAlignmentConstants.MAX_ROTATIONAL_SPEED;
                    if (Math.abs(rotationError) > Math.abs(lastValues[1])) {
                        rotationSpeed = 0;
                    }
                    lastValues[1] = rotationError;

                    forwardSpeed = Math.min(forwardSpeed, AutoAlignmentConstants.MAX_LINEAR_SPEED);
                    strafeSpeed = Math.min(strafeSpeed, AutoAlignmentConstants.MAX_LINEAR_SPEED);
                    rotationSpeed = Math.min(rotationSpeed, AutoAlignmentConstants.MAX_ROTATIONAL_SPEED);

                    ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                        forwardSpeed * AutoAlignmentConstants.SPEED_MULTIPLIER,
                        strafeSpeed * AutoAlignmentConstants.SPEED_MULTIPLIER,
                        rotationSpeed,
                        robotRotation
                    );

                    swerveDrive.runChassisSpeeds(speeds);
                }
            },
            interrupted -> swerveDrive.stop(),
            () -> {
                if (targetLocation == null) return true;
                double distanceToTarget = targetLocation.getDistance(localization.getRobotPose().getTranslation());
                return distanceToTarget > lastValues[0];
            },
            this, swerveDrive
        );
    }
}
