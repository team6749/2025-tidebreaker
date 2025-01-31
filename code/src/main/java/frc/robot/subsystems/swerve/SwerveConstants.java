package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class SwerveConstants {

    // Max linear velocity of the module (how fast it can spin)
    public static final LinearVelocity maxLinearVelocity = MetersPerSecond.of(4.5);

    // This should be based on physical properties of the the max wheel speeds in a circle
    public static final AngularVelocity maxAngularVelocity = DegreesPerSecond.of(360);

    public static final double angleReduction = (150.0 / 7.0);
    public static final double driveReduction = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

    public static final Distance wheelDiameter = Inches.of(4);
    public static final Distance wheelCircumference = wheelDiameter.times(Math.PI);

    // Distance between the center of wheels along the width of the robot(Y axis)
    public static final Distance trackWidth = Meters.of(0.5);
    // Distance between the center of wheels along the depth of the robot (X axis)
    public static final Distance trackHeight = Meters.of(0.7);

    public static final Translation2d[] moduleLocations = {
            // Front left
            new Translation2d(trackHeight.div(2), trackWidth.div(2)),
            // Front right
            new Translation2d(trackHeight.div(2), trackWidth.div(2).unaryMinus()),
            // Back left
            new Translation2d(trackHeight.div(2).unaryMinus(), trackWidth.div(2)),
            // Back right
            new Translation2d(trackHeight.div(2).unaryMinus(), trackWidth.div(2).unaryMinus()),
    };

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.moduleLocations);

}
