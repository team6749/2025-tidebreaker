// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int kDriverControllerPort = 0;
  }
  public static class SwerveConstants {
    public static final double WheelRadius = 2;
    public static final double GearRatio = 10000; //todo, put in an actual value
    public static final double turnMotorMaxOutputVolts = 7;
    public static final double angleKP = 0.04; //todo, put in actual values for all pids
    public static final double angleKI = 0;
    public static final double angleKD = 0;
    public static final double maxVelocity = 5;
    public static final double bodyHeading = 13; //todo, put in actual heading
  }
  public static class DrivingConstants {
    public static final double deadZone = 0.1;
  }
}
