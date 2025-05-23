// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Hertz;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class Constants {

    // How long each frame of the simulation is
    public static final Time simulationTimestep = Hertz.of(50).asPeriod();
    public static final int armMotorID = 42;
    public static final int elevatorMotorID = 18;
    public static final int clawMotorID = 54;
    public static final int algaeMotorID = 40;
    public static final int climberMotorPort = 20;
    public static final double deadZone = 0.06;

    public static final int kTopButtonBoardPort = 2;
    public static final int kBottomButtonBoardPort = 3;

    public static final Distance armClearance = Centimeters.of(3);
}
