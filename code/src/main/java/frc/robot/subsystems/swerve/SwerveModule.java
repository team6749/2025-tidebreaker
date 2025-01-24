package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.geometry.Translation2d;


public class SwerveModule {

    CANcoder encoder;


    /// Module Position Relative to the robot
    @NotLogged
    Translation2d position;


    SwerveModule(Translation2d position) {
        this.position = position;
    }

}
