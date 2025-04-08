// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoAlignSubsystem extends SubsystemBase {
  SwerveDrive swerveDrive;
  Localization localization;

  Pose2d ABReefFace = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? (new Pose2d(,,0)):(new Pose2d(,,0)));
  Pose2d CDReefFace = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? (new Pose2d(,,0)):(new Pose2d(,,0)));
  Pose2d EFReefFace = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? (new Pose2d(,,0)):(new Pose2d(,,0)));
  Pose2d GHReefFace = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? (new Pose2d(,,0)):(new Pose2d(,,0)));
  Pose2d IJReefFace = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? (new Pose2d(,,0)):(new Pose2d(,,0)));
  Pose2d KLReefFace = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue ? (new Pose2d(,,0)):(new Pose2d(,,0)));
  /** Creates a new AutoAlign. */
  public AutoAlignSubsystem(SwerveDrive containerSwerveDrive, Localization containerLocalization) {
    swerveDrive = containerSwerveDrive;
    localization = containerLocalization;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void compareVectors() {
    double ABNorm = localization.getRobotPose().relativeTo(ABReefFace).getTranslation().getNorm();
    double CDNorm = localization.getRobotPose().relativeTo(CDReefFace).getTranslation().getNorm();
    double EFNorm = localization.getRobotPose().relativeTo(EFReefFace).getTranslation().getNorm();
    double GHNorm = localization.getRobotPose().relativeTo(GHReefFace).getTranslation().getNorm();
    double IJNorm = localization.getRobotPose().relativeTo(IJReefFace).getTranslation().getNorm();
    double KLNorm = localization.getRobotPose().relativeTo(KLReefFace).getTranslation().getNorm();
  }
}
