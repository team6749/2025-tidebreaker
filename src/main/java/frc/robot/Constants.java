// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.PrimitiveIterator.OfDouble;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.SwerveModule;
import edu.wpi.first.distance;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int driverControllerPort = 0;
  }
  public static class SwerveConstants {
    public static final Distance WheelRadius = Inches.of(2);
    public static final Distance wheelCircumference = WheelRadius.times(Math.PI);
    //put in a real value\/\/\/\/\/
     //put in a real value\/\/\/\/\/
      //put in a real value\/\/\/\/\/
    public static final double GearRatio = 6.82; //todo, put in an actual value
     //put in a real value^^^^^^^
      //put in a real value^^^^^^
      //put in a real value^^^^^^^
    public static final double turnMotorMaxOutputVolts = 7;
    public static final double angleKP = 0.04; //todo, put in actual values for all pids
    public static final double angleKI = 0;
    public static final double angleKD = 0;
    public static final double maxVelocity = 5;
    public static final double bodyHeading = 13; //todo, put in actual heading
    public static final double XCenterDistanceIn = 13; 
    public static final double YCenterDistanceIn = 16; 
    public static class SwerveModules {
        public static final SwerveModule FLModule = new SwerveModule("FL",
         0
         , 0,
          new Translation2d(-XCenterDistanceIn,YCenterDistanceIn));
        public static final SwerveModule FRModule = new SwerveModule("FR",
         0
         , 0,
          new Translation2d(XCenterDistanceIn,YCenterDistanceIn));
        public static final SwerveModule BLModule = new SwerveModule("BL",
         0
         , 0,
          new Translation2d(-XCenterDistanceIn,-YCenterDistanceIn));
        public static final SwerveModule BRModule = new SwerveModule("BR",
         0
         , 0,
          new Translation2d(XCenterDistanceIn,-YCenterDistanceIn));

    public static final SwerveModule[] modules = {SwerveModules.FLModule, SwerveModules.FRModule, SwerveModules.BLModule, SwerveModules.BRModule};
}
    };
  public static class DrivingConstants {
    public static final double deadZone = 0.1;
  }
}
