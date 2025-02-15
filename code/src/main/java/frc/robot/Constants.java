// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;


/** Add your docs here. */
public class Constants {

    // How long each frame of the simulation is
    public static final Time simulationTimestep = Hertz.of(50).asPeriod();
    public static final int armMotorPort = 14;

    public static final double armGearRatio = ((1.0 / 5.0) * (1.0 / 5.0) * (16.0 / 30.0)); // gearbox:1/5, gearbox2:1/5, chain:16/30 no clue about chain, double check that
    public class ElevatorSetPoints {
        public static Distance intake = Meters.of(0);
        public static Distance l1 = Meters.of(0.3);
        public static Distance l2 = Meters.of(0.4);
        public static Distance l3 = Meters.of(0.5);
        public static Distance l4 = Meters.of(0.64);

    }
}
