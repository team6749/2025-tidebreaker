package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Millimeters;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;

public class SwerveConstants {

    public static final SlewRateLimiter driveLimiterX = new SlewRateLimiter(10);
    public static final SlewRateLimiter driveLimiterY = new SlewRateLimiter(10);
    public static final SlewRateLimiter driveLimiterTheta = new SlewRateLimiter(15);

    // Max linear velocity of the module (how fast it can spin)
    public static final LinearVelocity maxLinearVelocity = MetersPerSecond.of(3);

    // This should be based on physical properties of the the max wheel speeds in a
    // circle
    public static final AngularVelocity maxAngularVelocity = DegreesPerSecond.of(90);

    public static final double angleReduction = (150.0 / 7.0);
    public static final double driveReduction = 6.82;

    public static final Distance wheelDiameter = Millimeters.of(100);
    public static final Distance wheelCircumference = wheelDiameter.times(Math.PI);

    // Distance between the center of wheels along the width of the robot(Y axis)
    public static final Distance trackWidth = Meters.of(0.6858);
    // Distance between the center of wheels along the depth of the robot (X axis)
    public static final Distance trackHeight = Meters.of(0.5334);

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

    public static int FLAngleMotorPort = 1;
    public static int FLEncoderPort = 2;
    public static int FLDriveMotorPort = 3;

    public static int FRAngleMotorPort = 4;
    public static int FREncoderPort = 5;
    public static int FRDriveMotorPort = 6;

    public static int BRAngleMotorPort = 7;
    public static int BREncoderPort = 8;
    public static int BRDriveMotorPort = 9;

    public static int BLAngleMotorPort = 10;
    public static int BLEncoderPort = 11;
    public static int BLDriveMotorPort = 12;
}
