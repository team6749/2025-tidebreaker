// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.Robot;

@Logged
public class Elevator extends SubsystemBase {

  // Constants
  public static Distance minHeight = Meters.of(0);
  public static Distance maxHeight = Meters.of(1.45);
  public static int gearboxRatio = 25;
  public static double carriageWeightKg = 15;
  public static Distance sproketDiameter = Inches.of(3);
  // Total Ratio of the mechanism from rotations to meters
  public static double outputRatio = (1.0 / gearboxRatio) * sproketDiameter.in(Meters) * Math.PI;
  public static Distance toleranceOnReachedGoal = Meters.of(0.02);

  public static LinearVelocity maxVelocity = MetersPerSecond.of(1);
  public static LinearAcceleration maxAccerleration = MetersPerSecondPerSecond.of(0.5);

  // Hardware
  private final TalonFX m_motor = new TalonFX(1);

  private final PIDController m_controller = new PIDController(
      4,
      0,
      0);
  ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
      0,
      1.15, // Most important value
      5,
      0);

  // private final ExponentialProfile m_profile =
  // new ExponentialProfile(
  // ExponentialProfile.Constraints.fromCharacteristics(
  // Constants.kElevatorMaxV, Constants.kElevatorkV, Constants.kElevatorkA));

  // TODO convert from using a trapezoidal profile to an ExponentialProfile based on sysid
  TrapezoidProfile m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxVelocity.in(MetersPerSecond), maxAccerleration.in(MetersPerSecondPerSecond)));

  // Intermediate goal position, used for closed loop control.
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State(minHeight.in(Meters), 0);

  BooleanSupplier limitSwitch;
  boolean isHomed = false;

  // Simulation Control
  private final TalonFXSimState motorSim = m_motor.getSimState();

  // Pyhsically based model of the elevator
  private final DCMotor m_elevatorGearbox = DCMotor.getFalcon500(1);
  private final ElevatorSim m_elevatorSim = new ElevatorSim(
      m_elevatorGearbox,
      gearboxRatio,
      carriageWeightKg,
      sproketDiameter.in(Meters),
      minHeight.in(Meters),
      maxHeight.in(Meters),
      true,
      maxHeight.in(Meters), // Start at min height
      0.00,
      0.0);

  // Mechanism2d visualization
  private final Mechanism2d m_mech2d = new Mechanism2d(2, maxHeight.in(Meters));
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 1, minHeight.in(Meters));
  private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
      new MechanismLigament2d("Elevator", minHeight.in(Meters), 90));

  /** Creates a new Elevator. */
  public Elevator() {
    setDefaultCommand(holdPositionAtSetpointCommand());

    if(Robot.isSimulation()) {
      limitSwitch = () -> m_elevatorSim.wouldHitLowerLimit(getPosition().in(Meters));
    } else {
      // TODO implement real limit switch
    }

    SmartDashboard.putData("Elevator Sim", m_mech2d);
  }

  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(m_motor.getVelocity().getValue().in(RotationsPerSecond) * outputRatio);
  }

  public Distance getPosition() {
    return Meters.of(m_motor.getPosition().getValue().in(Rotations) * outputRatio);
  }

  public boolean getLimitSwitch () {
    return limitSwitch.getAsBoolean();
  }

  // Runs the elevator if it is homed
  public void runGoal(Distance goal) {
    if (isHomed == false) {
      return;
    }
    var goalState = new TrapezoidProfile.State(goal.in(Meters), 0);
    var next = m_profile.calculate(Constants.simulationTimestep.in(Seconds), m_setpoint, goalState);

    // With the setpoint value we run PID control like normal
    double pidOutput = m_controller.calculate(getPosition().in(Meters), m_setpoint.position);
    double feedforwardOutput = m_feedforward.calculateWithVelocities(m_setpoint.velocity, next.velocity);
    runVolts(Volts.of(pidOutput + feedforwardOutput));

    m_setpoint = next;
  }

  // runs the elevator with a specific voltage. Does not care about home state
  public void runVolts(Voltage volts) {
    m_motor.setVoltage(volts.in(Volts));
    // System.out.println(volts.in(Volts));
    if (Robot.isSimulation()) {
      m_elevatorSim.setInput(volts.in(Volts));
    }
  }

  public void stop() {
    m_motor.setVoltage(0);
    m_elevatorSim.setInput(0);
  }

  @Override
  public void periodic() {
    if(isHomed == false) {
      runVolts(Volts.of(-1));
      if(getLimitSwitch()) {
        isHomed = true;
        m_motor.setPosition(minHeight.in(Meters) / outputRatio);
      }
    }

    // Telemetry
    m_elevatorMech2d.setLength(getPosition().in(Meters));
  }

  @Override
  public void simulationPeriodic() {
    m_elevatorSim.update(Constants.simulationTimestep.in(Seconds));
    motorSim.setRawRotorPosition(m_elevatorSim.getPositionMeters() / outputRatio);
    motorSim.setRotorVelocity(m_elevatorSim.getVelocityMetersPerSecond() / outputRatio);
  }

  // TODO create a logging class to log the Profile States
  public double getSetpointPosition() {
    return m_setpoint.position;
  }

  public double getSetpointVelocity() {
    return m_setpoint.velocity;
  }

  public void resetProfileState() {
    // resets the profile to
    m_setpoint = new TrapezoidProfile.State(getPosition().in(Meters), getVelocity().in(MetersPerSecond));
  }

  public boolean atGoalPosition(Distance position) {
    return getPosition().isNear(position, toleranceOnReachedGoal);
  }

  // Holds position at the current measured location
  // Note, elevator hold can possibly be unstable when it is started at a high
  // speed
  // or if the elevator was previously controlled by an open loop. Always call
  // resetProfileState
  // after any open loop command
  public Command holdPositionAtSetpointCommand() {
    return Commands.startRun(() -> {
      System.out.println("Elevator hold started");
      if (getPosition().isNear(Meters.of(m_setpoint.position), toleranceOnReachedGoal) == false) {
        System.out.println(
            "WARNING: Elevator Hold started far away from setpoint, resetting setpoint to avoid rapid movement");
        m_setpoint = new TrapezoidProfile.State(getPosition().in(Meters), 0);
        // If we are near the setpoint, we should use that, otherwise, just use the
        // current hold position.
      }
    }, () -> {
      runGoal(Meters.of(m_setpoint.position));
    }, this).finallyDo(() -> {
      stop();
    });
  }

  // Runs the elevator to a specific position and then holds there, until the
  // command is interrupted
  public Command goToPositionCommand(Distance position) {
    return Commands.startRun(() -> {
      resetProfileState();
    }, () -> {
      System.out.println(position.in(Meters));
      runGoal(position);
    }, this).until(() -> atGoalPosition(position));
  }

}
