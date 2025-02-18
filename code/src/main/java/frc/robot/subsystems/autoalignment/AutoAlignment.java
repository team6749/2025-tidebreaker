package frc.robot.subsystems.autoalignment;

import edu.wpi.first.wpilibj2.command.Command;
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

    public AutoAlignment(Localization localization) {
        this.localization = localization;
        this.autoAlignmentLoader = new AutoAlignmentLoader();
    }

    @Override
    public void periodic() {
        // localization.getDashboardField().getObject("testingA").setPose(coralSproutPositions.get("A"));
        // localization.getDashboardField().getObject("testingB").setPose(coralSproutPositions.get("B"));
        // localization.getDashboardField().getObject("testingC").setPose(coralSproutPositions.get("C"));
        // localization.getDashboardField().getObject("testingD").setPose(coralSproutPositions.get("D"));
        // localization.getDashboardField().getObject("testingE").setPose(coralSproutPositions.get("E"));
        // localization.getDashboardField().getObject("testingF").setPose(coralSproutPositions.get("F"));
        // localization.getDashboardField().getObject("testingG").setPose(coralSproutPositions.get("G"));
        // localization.getDashboardField().getObject("testingH").setPose(coralSproutPositions.get("H"));
        // localization.getDashboardField().getObject("testingI").setPose(coralSproutPositions.get("I"));
        // localization.getDashboardField().getObject("testingJ").setPose(coralSproutPositions.get("J"));
        // localization.getDashboardField().getObject("testingK").setPose(coralSproutPositions.get("K"));
        // localization.getDashboardField().getObject("testingL").setPose(coralSproutPositions.get("L"));
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

    public static class AutoAlignCommand extends Command {
        private final AutoAlignment autoAlignment;
        private final SwerveDrive swerveDrive;
        private final Input inputSubsystem;
        private Translation2d targetLocation;
        private Rotation2d targetRotation;
        private double lastDistance = Double.MAX_VALUE;
        private double lastRotationError = Double.MAX_VALUE;


        public AutoAlignCommand(AutoAlignment autoAlignment, SwerveDrive swerveDrive, Input inputSubsystem) {
            this.autoAlignment = autoAlignment;
            this.swerveDrive = swerveDrive;
            this.inputSubsystem = inputSubsystem;
            addRequirements(autoAlignment);
        }

        @Override
        public void initialize() {
            String coralSproutName = autoAlignment.findNearestReefTarget();
            Pose2d coralSproutPose = autoAlignment.findNearestReefPositionBySproutName(coralSproutName);
            
            targetLocation = coralSproutPose.getTranslation();
            targetRotation = coralSproutPose.getRotation();
            lastDistance = Double.MAX_VALUE;
            lastRotationError = Double.MAX_VALUE;
        }

        @Override
        public void execute() {
            if (targetLocation != null) {
                Pose2d robotPose = autoAlignment.localization.getRobotPose();
                Translation2d robotPosition = robotPose.getTranslation();
                Rotation2d robotRotation = robotPose.getRotation();

                Translation2d movementVector = targetLocation.minus(robotPosition);
                double distanceToTarget = movementVector.getNorm();
                double rotationError = targetRotation.minus(robotRotation).getRadians();

                // 🔹 Stop if moving away from target
                if (distanceToTarget > lastDistance) {
                    end(false);
                    return;
                }
                lastDistance = distanceToTarget; // Store last distance

                // 🔹 Normalize movement vector to get correct X and Y speeds
                Translation2d movementDirection = movementVector.div(distanceToTarget); // Normalize to unit vector

                // 🔹 Deceleration: Slow down as we get close to target
                double speedFactor = Math.min(distanceToTarget * AutoAlignmentConstants.DECELERATION_FACTOR, AutoAlignmentConstants.DECELERATION_LIMIT);

                // 🔹 Compute movement speeds in the correct direction
                double forwardSpeed = movementDirection.getX() * speedFactor;
                double strafeSpeed = movementDirection.getY() * speedFactor;

                // 🔹 Rotation Control: Keep turning until we overshoot the goal
                double rotationSpeed = rotationError * AutoAlignmentConstants.MAX_ROTATIONAL_SPEED;
                if (Math.abs(rotationError) > Math.abs(lastRotationError)) {
                    rotationSpeed = 0; // Stop rotating if we overshoot
                }
                lastRotationError = rotationError;

                // 🔹 Apply a speed cap to prevent excessive acceleration
                forwardSpeed = Math.min(forwardSpeed, AutoAlignmentConstants.MAX_LINEAR_SPEED);
                strafeSpeed = Math.min(strafeSpeed, AutoAlignmentConstants.MAX_LINEAR_SPEED);
                rotationSpeed = Math.min(rotationSpeed, AutoAlignmentConstants.MAX_ROTATIONAL_SPEED);

                // 🔹 Convert speeds to field-relative movement
                ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    forwardSpeed * AutoAlignmentConstants.SPEED_MULTIPLIER, 
                    strafeSpeed * AutoAlignmentConstants.SPEED_MULTIPLIER, 
                    rotationSpeed,
                    robotRotation
                );

                swerveDrive.runChassisSpeeds(speeds);
            }
        }

        @Override
        public boolean isFinished() {
            if (targetLocation == null) {
                return true;
            }
        
            Pose2d robotPose = autoAlignment.localization.getRobotPose();
            Translation2d robotPosition = robotPose.getTranslation();
        
            double distanceToTarget = targetLocation.getDistance(robotPosition);
        
            // 🔹 Stop when movement moves away OR user control
            return (distanceToTarget > lastDistance || inputSubsystem.isUserControlActive());
        }
        
        @Override
        public void end(boolean interrupted) {
            swerveDrive.stop();
        }
    }
}
