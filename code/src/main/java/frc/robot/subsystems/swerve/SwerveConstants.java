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
    public static final LinearVelocity maxLinearVelocity = MetersPerSecond.of(4);

    // This should be based on physical properties of the the max wheel speeds in a circle
    public static final AngularVelocity maxAngularVelocity = DegreesPerSecond.of(360);

    public static final double angleReduction = (150.0 / 7.0);
    public static final double driveReduction = 6.82;

    public static final Distance wheelDiameter = Inches.of(4);
    public static final Distance wheelCircumference = wheelDiameter.times(Math.PI);

    // Distance between the center of wheels along the width of the robot(Y axis)
    public static final Distance trackWidth = Meters.of(0.6858);
    // Distance between the center of wheels along the depth of the robot (X axis)
    public static final Distance trackHeight = Meters.of(0.5334);

    public static final Translation2d[] moduleLocations = {
            // Front left
            new Translation2d(trackHeight.div(2), trackWidth.div(2)),
            // Front right
            new Translation2d(trackHeight.div(2).unaryMinus(), trackWidth.div(2)),
            // Back left
            new Translation2d(trackHeight.div(2), trackWidth.div(2).unaryMinus()),
            // Back right
            new Translation2d(trackHeight.div(2).unaryMinus(), trackWidth.div(2).unaryMinus()),
    };

    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.moduleLocations);

    public static int FRDriveMotorPort = ;
    public static int FRAngleMotorPort = ;
    public static int FREncoderPort = ;

    public static int FLDriveMotorPort = ;
    public static int FLAngleMotorPort = ;
    public static int FLEncoderPort = ;

    public static int BLDriveMotorPort = ;
    public static int BLAngleMotorPort = ;
    public static int BLEncoderPort = ;

    public static int BRDriveMotorPort = ;
    public static int BRAngleMotorPort = ;
    public static int BREncoderPort = ;
}
