// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;


/**
 * The Constants class stores global constants used throughout the robot code.
 * 
 * <p>This class contains configuration values that remain constant during 
 * runtime, such as simulation timing, dead zones, and predefined elevator 
 * setpoints.
 * 
 * <p>Features:
 * <ul>
 *   <li>Defines the simulation timestep for physics calculations</li>
 *   <li>Specifies a controller dead zone to prevent unintentional input</li>
 *   <li>Provides predefined elevator positions for different levels</li>
 * </ul>
 * 
 * This class follows the convention of organizing constants in a single location 
 * to improve maintainability and readability.
 */
public class Constants {

    // How long each frame of the simulation is
    public static final Time simulationTimestep = Hertz.of(50).asPeriod();
    public static final double deadZone = 0.06;

    public class ElevatorSetPoints {
        public static Distance intake = Meters.of(0);
        public static Distance l1 = Meters.of(0.3);
        public static Distance l2 = Meters.of(0.4);
        public static Distance l3 = Meters.of(0.5);
        public static Distance l4 = Meters.of(0.64);

    }
}
