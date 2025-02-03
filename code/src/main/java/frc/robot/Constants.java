// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Hertz;

import edu.wpi.first.units.measure.Time;

/** Add your docs here. */
public class Constants {

    // How long each frame of the simulation is
    public static final Time simulationTimestep = Hertz.of(50).asPeriod();
    public static final int armMotorPort = 100000;

    public static final double armGearRatio = (1/5) * (1/5) * (16/30); // gearbox:1/5, gearbox2:1/5, chain:16/30 no clue about chain, double check that
}
