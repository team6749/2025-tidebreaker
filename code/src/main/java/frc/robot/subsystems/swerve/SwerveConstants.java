package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.Distance;

public class SwerveConstants {

        static double angleReduction = (150.0 / 7.0);
        static double driveReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

        static Distance wheelDiameter = Inches.of(4);
        static Distance wheelCircumference = wheelDiameter.times(Math.PI);

    // Distance between the center of wheels along the width of the robot(Y axis)
    static Distance trackWidth = Meters.of(0.4);
    // Distance between the center of wheels along the depth of the robot (X axis)
    static Distance trackHeight = Meters.of(0.6);

    static SwerveModule frontLeft = new SwerveModule(new Translation2d(trackHeight.div(2), trackWidth.div(2)));
    static SwerveModule frontRight = new SwerveModule(
            new Translation2d(trackHeight.div(2), trackWidth.div(2).unaryMinus()));
    static SwerveModule backLeft = new SwerveModule(
            new Translation2d(trackHeight.div(2), trackWidth.div(2).unaryMinus()));
    static SwerveModule backRight = new SwerveModule(
            new Translation2d(trackHeight.div(2).unaryMinus(), trackWidth.div(2).unaryMinus()));

    static SwerveModule[] modules = {
            SwerveConstants.frontLeft,
            SwerveConstants.frontRight,
            SwerveConstants.backLeft,
            SwerveConstants.backRight
    };

    static Translation2d[] moduleLocations = {
            modules[0].position,
            modules[1].position,
            modules[2].position,
            modules[3].position,
    };

    static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.moduleLocations);

}
