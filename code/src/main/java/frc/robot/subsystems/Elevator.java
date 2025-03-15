// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Centimeters;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

@Logged
public class Elevator extends SubsystemBase {

  public static Distance minHeight = Meters.of(0);
  public static Distance maxHeight = Meters.of(0.665);
  public static Distance simStartHeight = Meters.of(0.665);
  public static double gearboxRatio = (4.0 / 1.0) * (5.0 / 1.0);
  public static Mass carriageMass = Kilograms.of(4);
  public static Distance sprocketDiameter = Centimeters.of(5.3);
  public BooleanSupplier limitSwitch;
  // Total Ratio for elevator motor in meters
  public static double outputRatio = (1.0 / gearboxRatio) * sprocketDiameter.in(Meters) * Math.PI;
  public static Distance toleranceOnReachedGoal = Centimeters.of(2);
  public static TalonFX elevatorMotor = new TalonFX(18);
  public static LinearVelocity maxVelocity = MetersPerSecond.of(0.7);
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

  private TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(
          maxVelocity.in(MetersPerSecond),
          maxAcceleration.in(MetersPerSecondPerSecond)));

  private TrapezoidProfile.State targetState = new State(0, 0);
  private TrapezoidProfile.State setpointState = new State(0, 0);

  private boolean isHomed = false;
  private boolean closedLoop = false;
  private boolean motorInverted = false;

  private PIDController elevatorPID = new PIDController(20, 0, 0);
  private ElevatorFeedforward feedforward = new ElevatorFeedforward(0, 0.20, 13);

  @SuppressWarnings("removal")
  public Elevator() {
    stop();
    elevatorMotor.setInverted(motorInverted);
    var angleMotorCurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(80))
        .withStatorCurrentLimitEnable(true);
    elevatorMotor.getConfigurator().apply(angleMotorCurrentLimits);

    if (Robot.isSimulation()) {
      limitSwitch = () -> getPosition().isNear(minHeight, Meters.of(0.01));
    } else {
      limitSwitch = () -> true;
    }
    SmartDashboard.putData("Elevator Sim", mech2d);
  }

  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid
  // reallocation.
  private final MutDistance m_distance = Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid
  // reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);

  private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
      // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
      new SysIdRoutine.Config(),
      new SysIdRoutine.Mechanism(
          // Tell SysId how to plumb the driving voltage to the motors.
          voltage -> {
            runVolts(voltage);
          },
          // Tell SysId how to record a frame of data for each motor on the mechanism
          // being
          // characterized.
          log -> {
            // Record a frame for the left motors. Since these share an encoder, we consider
            // the entire group to be one motor.
            log.motor("elevator")
                .voltage(
                    m_appliedVoltage.mut_replace(
                        elevatorMotor.getMotorVoltage().getValueAsDouble(),
                        Volts))
                .linearPosition(m_distance.mut_replace(getPosition().in(Meters), Meters))
                .linearVelocity(
                    m_velocity.mut_replace(getVelocity().in(MetersPerSecond),
                        MetersPerSecond));
            // Record a frame for the right motors. Since these share an encoder, we
            // consider
            // the entire group to be one motor.
          },
          (Subsystem) // Tell SysId to make generated commands require this subsystem, suffix test
          this));

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
      var next = trapezoidProfile.calculate(Constants.simulationTimestep.in(Seconds), setpointState, targetState);
      Voltage PIDOutput = Volts.of(elevatorPID.calculate(getPosition().in(Meters), setpointState.position));
      Voltage feedForwardOutput = Volts.of(feedforward.calculate(next.velocity));
      setVolts(PIDOutput.plus(feedForwardOutput));
      setpointState = next;
    }
    elevatorMech2d.setLength(getPosition().in(Meters));
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

  private void runClosedLoopSetGoal(Distance goal) {
    targetState = new TrapezoidProfile.State(goal.in(Meters), 0);
    closedLoop = true;
  }

  public void resetProfileState() {
    setpointState = new TrapezoidProfile.State(getPosition().in(Meters), getVelocity().in(MetersPerSecond));
  }

  public Command runOpenLoopCommand(Voltage Volts) {
    return Commands.runEnd(() -> runVolts(Volts), () -> stop(), this);
  }

  public Command goToPositionCommand(Distance targetHeight) {
    if (targetHeight.gt(maxHeight)) {
      Exception exception = new Exception(
          "WARNING: Elevator instructed to go higher than max height. " + targetHeight.toString());
      System.out.println(exception.getMessage());
      exception.printStackTrace();
      if (Robot.isSimulation()) {
        // TODO: Crash the simulator
      }
      return goToPositionCommandPrivate(maxHeight);
    }
    if (targetHeight.lt(minHeight)) {
      Exception exception = new Exception(
          "WARNING: Elevator instructed to go lower than min height. " + targetHeight.toString());
      System.out.println(exception.getMessage());
      exception.printStackTrace();
      if (Robot.isSimulation()) {
        // TODO: Crash the simulator
      }
      return goToPositionCommandPrivate(minHeight);
    }
    return goToPositionCommandPrivate(targetHeight);
  }

  private Command goToPositionCommandPrivate(Distance targetHeight) {
    return Commands.startRun(() -> {
      resetProfileState();
    }, () -> {
      runClosedLoopSetGoal(targetHeight);
    }, this).until(() -> isAtTarget(targetHeight)).handleInterrupt(() -> {
      System.out.println("WARNING: Elevator go to position command interrupted. Holding Current Position");
      targetState = new TrapezoidProfile.State(getPosition().in(Meters), 0);
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

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction).until(
        () -> direction == SysIdRoutine.Direction.kForward ? getPosition().in(Meters) > maxHeight.in(Meters) - 0.02
            : getPosition().in(Meters) < minHeight.in(Meters) + 0.02);
  }
}
