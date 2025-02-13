// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
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
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Elevator extends SubsystemBase {
  // TODO: input real values
  boolean isHomed = false;
  boolean closedLoop = false;
  boolean openLoop;
  public static Distance minHeight = Meters.of(0);
  public static Distance maxHeight = Meters.of(0.65);
  public static Distance simStartHeight = Meters.of(0.1);
  public static double gearboxRatio = 20;
  public static Mass carriageMass = Kilograms.of(4);
  public static Distance sprocketDiameter = Inches.of(2.5);
  public BooleanSupplier limitSwitch; 
  // Total Ratio for elevator motor in meters
  public static double outputRatio = (1.0 / gearboxRatio) * sprocketDiameter.in(Meters) * Math.PI;
  public static Distance toleranceOnReachedGoal = Meters.of(0.04);
  public static TalonFX elevatorMotor = new TalonFX(11);

  public static LinearVelocity maxVelocity = MetersPerSecond.of(1);
  public static LinearAcceleration maxAcceleration = MetersPerSecondPerSecond.of(1);

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
      0.0,0
      );

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
  TrapezoidProfile.State currentState = new State(getPosition().in(Meters), getVelocity().in(MetersPerSecond));
  TrapezoidProfile.State desiredState;

  PIDController elevatorPID = new PIDController(0, 0, 0);
  ElevatorFeedforward feedforward = new ElevatorFeedforward(1, 0, 0);

  public Elevator() {
    if (Robot.isSimulation()) {
      limitSwitch = () -> getPosition().isNear(minHeight, Meters.of(0.01));
    }
    else {
      //todo
    }
    SmartDashboard.putData("Elevator Sim", mech2d);
  }

  @Override
  public void periodic() {
    if (isHomed == false) {
      setVolts(-2);
      if (getLimitSwitch()) {
        isHomed = true;
        elevatorMotor.setPosition(minHeight.in(Meters) / outputRatio);
      }
    }

    if (closedLoop) {
      if (isHomed == false) {
        System.out.println("WARNING: Elevator not homed, but closed loop control is enabled");
        resetProfileState();
        return;
      }
      var next = trapezoidProfile.calculate(Constants.simulationTimestep.in(Seconds), currentState, desiredState);
      Voltage PIDOutput = Volts.of(elevatorPID.calculate(getPosition().in(Meters), desiredState.position));
      Voltage feedForwardOutput = Volts.of(feedforward.calculate(next.velocity));
      setVolts(PIDOutput.in(Volts) + feedForwardOutput.in(Volts));
    }
    if(openLoop) {
      setVolts(0.6);
    }
    elevatorMech2d.setLength(getPosition().in(Meters)); 
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
      simElevator.update(Constants.simulationTimestep.in(Seconds));
      simMotor.setRawRotorPosition(simElevator.getPositionMeters() / outputRatio);
      simMotor.setRotorVelocity(simElevator.getVelocityMetersPerSecond() / outputRatio);
  }

  public Distance getPosition() {
    return Meters.of(elevatorMotor.getPosition().getValueAsDouble() * outputRatio);
  }

  public LinearVelocity getVelocity() {
    return MetersPerSecond.of(elevatorMotor.getVelocity().getValueAsDouble() * outputRatio);
  }

  public void setVolts(double voltInput) {
    if (RobotBase.isSimulation()) {
      simElevator.setInputVoltage(voltInput);
    } else {
      elevatorMotor.setVoltage(voltInput);
    }
  }

  public void runClosedLoopSetGoal(Distance goal) {
    desiredState = new TrapezoidProfile.State(goal.in(Meters), 0);
    closedLoop = true;
  }

  public void resetProfileState() {
    currentState = new TrapezoidProfile.State(getPosition().in(Meters), getVelocity().in(MetersPerSecond));
  }

  public boolean isAtTarget(Distance position) {
    return getPosition().isNear(position, toleranceOnReachedGoal);
  }

  public Command goToPositionCommand(Distance position) {
    return Commands.startRun(() -> {
      resetProfileState();
    }, () -> {
      runClosedLoopSetGoal(position);
    }, this).until(() -> isAtTarget(position)).handleInterrupt(() -> {
      System.out.println("WARNING: Elevator go to position command interrupted. Holding Current Position");
      desiredState = new TrapezoidProfile.State(getPosition().in(Meters), 0);
    });
  }

  public boolean getLimitSwitch() {
    return limitSwitch.getAsBoolean();
  }

  public void stop() {
    elevatorMotor.setVoltage(0);
    simElevator.setInputVoltage(0);
  }
}
