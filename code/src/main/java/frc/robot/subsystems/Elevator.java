// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
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
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

@Logged
public class Elevator extends SubsystemBase {
  boolean isHomed = false;
  boolean closedLoop = false;
  boolean motorInverted = false;
  public static Distance minHeight = Meters.of(0);
  public static Distance maxHeight = Meters.of(0.65);
  public static Distance simStartHeight = Meters.of(0.65);
  public static double gearboxRatio = 20 * (26/32);
  public static Mass carriageMass = Kilograms.of(4);
  public static Distance sprocketDiameter = Centimeters.of(0);
  public BooleanSupplier limitSwitch; 
  // Total Ratio for elevator motor in meters
  public static double outputRatio = (1.0 / gearboxRatio) * sprocketDiameter.in(Meters)  * Math.PI;
  public static Distance toleranceOnReachedGoal = Meters.of(0.015);
  public static TalonFX elevatorMotor = new TalonFX(18);
  public static LinearVelocity maxVelocity = MetersPerSecond.of(0.2);
  public static LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(0.5);

  private final TalonFXSimState simMotor = elevatorMotor.getSimState();
  private final DCMotor elevatorGearbox = DCMotor.getFalcon500(1);

  private final ElevatorSim simElevator = new ElevatorSim(
      elevatorGearbox,
      gearboxRatio,
      carriageMass.in(Kilograms),
      sprocketDiameter.in(Meters),
      minHeight.in(Meters),
      maxHeight.in(Meters),
      true,
      simStartHeight.in(Meters),
      0,
      0);

  private final Mechanism2d mech2d = new Mechanism2d(2,
      maxHeight.in(Meters));
  private final MechanismRoot2d mech2dRoot = mech2d.getRoot("Elevator Root", 1,
      minHeight.in(Meters));
  private final MechanismLigament2d elevatorMech2d = mech2dRoot.append(
      new MechanismLigament2d("Elevator",
          simStartHeight.in(Meters),
          90));

  TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
          maxVelocity.in(MetersPerSecond),
          maxAcceleration.in(MetersPerSecondPerSecond)));
  TrapezoidProfile.State targetState = new State(0, 0);
  TrapezoidProfile.State endState = new State(0, 0);

  PIDController elevatorPID = new PIDController(0, 0, 0);
  ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0.1, 2);

  public Elevator() {
    stop();
    elevatorMotor.setInverted(motorInverted);
    if (Robot.isSimulation()) {
      limitSwitch = () -> getPosition().isNear(minHeight, Meters.of(0.01));
    } else {
      limitSwitch = () -> true;
    }
    SmartDashboard.putData("Elevator Sim", mech2d);
  }

  @Override
  public void periodic() {
    if (isHomed == false) {
      setVolts(Volts.of(-0.1));
      if (getLimitSwitch()) {
        isHomed = true;
        elevatorMotor.setPosition(minHeight.in(Meters) / outputRatio);
        stop();
      }
    }

    if (closedLoop) {
      if (isHomed == false) {
        System.out.println("WARNING: Elevator not homed, but closed loop control is enabled");
        resetProfileState();
        return;
      }
      var next = trapezoidProfile.calculate(Constants.simulationTimestep.in(Seconds), targetState, endState);
      Voltage PIDOutput = Volts.of(elevatorPID.calculate(getPosition().in(Meters), targetState.position));
      Voltage feedForwardOutput = Volts.of(feedforward.calculate(next.velocity));
      setVolts(PIDOutput.plus(feedForwardOutput));
      targetState = next;
    }
    elevatorMech2d.setLength(getPosition().in(Meters));
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    simElevator.update(Constants.simulationTimestep.in(Seconds));
    simMotor.setRawRotorPosition(simElevator.getPositionMeters() / outputRatio * (motorInverted ? -1 : 1));
    simMotor.setRotorVelocity(simElevator.getVelocityMetersPerSecond() / outputRatio * (motorInverted ? -1 : 1));
  }

  public boolean isAtTarget(Distance position) {
    return getPosition().isNear(position, toleranceOnReachedGoal);
  }

  public Distance getPosition() {
    return Meters.of(elevatorMotor.getPosition().getValueAsDouble() * outputRatio);
  }

  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(elevatorMotor.getVelocity().getValueAsDouble() * outputRatio);
  }

  private void setVolts(Voltage voltInput) {
    if (RobotBase.isSimulation()) {
      simElevator.setInputVoltage(voltInput.in(Volts));
    } else {
      elevatorMotor.setVoltage(voltInput.in(Volts));
    }
  }

  public void runVolts(Voltage voltInput) {
    closedLoop = false;
    setVolts(voltInput);
  }

  public void runClosedLoopSetGoal(Distance goal) {
    endState = new TrapezoidProfile.State(goal.in(Meters), 0);
    closedLoop = true;
  }

  public void resetProfileState() {
    targetState = new TrapezoidProfile.State(getPosition().in(Meters), getVelocity().in(MetersPerSecond));
  }

  public Command runOpenLoopCommand(Voltage Volts) {
    return Commands.runEnd(() -> runVolts(Volts), () -> stop(), this);
  }

  public Command goToPositionCommand(Distance position) {
    return Commands.startRun(() -> {
      resetProfileState();
    }, () -> {
      runClosedLoopSetGoal(position);
    }, this).until(() -> isAtTarget(position)).handleInterrupt(() -> {
      System.out.println("WARNING: Elevator go to position command interrupted. Holding Current Position");
      endState = new TrapezoidProfile.State(getPosition().in(Meters), 0);
    });
  }

  public boolean getLimitSwitch() {
    return limitSwitch.getAsBoolean();
  }

  public void stop() {
    elevatorMotor.setVoltage(0);
    simElevator.setInputVoltage(0);
    closedLoop = false;
  }
}
