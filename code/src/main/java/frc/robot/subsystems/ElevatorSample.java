package frc.robot.subsystems;


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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

@Logged
public class ElevatorSample extends SubsystemBase {

  // Constants
  public static Distance minHeight = Meters.of(0);
  public static Distance maxHeight = Meters.of(1.45);
  public static Distance simStartHeight = Meters.of(1);
  public static int gearboxRatio = 20;
  public static Mass carriageMass = Kilograms.of(15);
  public static Distance sproketDiameter = Inches.of(3);
  // Total Ratio of the mechanism from rotations to meters
  public static double outputRatio = (1.0 / gearboxRatio) * sproketDiameter.in(Meters) * Math.PI;
  public static Distance toleranceOnReachedGoal = Meters.of(0.02);

  public static LinearVelocity maxVelocity = MetersPerSecond.of(1);
  public static LinearAcceleration maxAccerleration = MetersPerSecondPerSecond.of(1);

  // Hardware
  private final TalonFX m_motor = new TalonFX(1);

  private final PIDController controller = new PIDController(
      20,
      0,
      0);
  private final ElevatorFeedforward feedforward = new ElevatorFeedforward(
      0,
      1.25, // Most important value
      5,
      0);

  // private final ExponentialProfile m_profile =
  // new ExponentialProfile(
  // ExponentialProfile.Constraints.fromCharacteristics(
  // Constants.kElevatorMaxV, Constants.kElevatorkV, Constants.kElevatorkA));

  // TODO convert from using a trapezoidal profile to an ExponentialProfile based
  // on sysid
  private final TrapezoidProfile profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxVelocity.in(MetersPerSecond), maxAccerleration.in(MetersPerSecondPerSecond)));

  // Intermediate goal position, used for closed loop control.
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State goalState = setpoint;
  private boolean isRunningClosedLoop = false;

  private BooleanSupplier limitSwitch;
  private boolean isHomed = false;

  // Simulation Control
  private final TalonFXSimState motorSim = m_motor.getSimState();

  // Pyhsically based model of the elevator
  private final DCMotor m_elevatorGearbox = DCMotor.getFalcon500(1);
  private final ElevatorSim m_elevatorSim = new ElevatorSim(
      m_elevatorGearbox,
      gearboxRatio,
      carriageMass.in(Kilograms),
      sproketDiameter.in(Meters),
      minHeight.in(Meters),
      maxHeight.in(Meters),
      true,
      simStartHeight.in(Meters),
      0.00,
      0.0);

  // Mechanism2d visualization
  private final Mechanism2d m_mech2d = new Mechanism2d(2, maxHeight.in(Meters));
  private final MechanismRoot2d m_mech2dRoot = m_mech2d.getRoot("Elevator Root", 1, minHeight.in(Meters));
  private final MechanismLigament2d m_elevatorMech2d = m_mech2dRoot.append(
      new MechanismLigament2d("Elevator", simStartHeight.in(Meters), 90));

  /** Creates a new Elevator. */
  public ElevatorSample() {
    if (Robot.isSimulation()) {
      limitSwitch = () -> getPosition().isNear(minHeight, Meters.of(0.01));
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

  public boolean getLimitSwitch() {
    return limitSwitch.getAsBoolean();
  }

  public void setGoalAndEnableClosedLoop(Distance goal) {
    goalState = new TrapezoidProfile.State(goal.in(Meters), 0);
    isRunningClosedLoop = true;
  }

  public void runOpenLoop(Voltage volts) {
    isRunningClosedLoop = false;
    setVolts(volts);
  }

  private void setVolts(Voltage volts) {
    m_motor.setVoltage(volts.in(Volts));
    if (Robot.isSimulation()) {
      m_elevatorSim.setInput(volts.in(Volts));
    }
  }

  // Stops motors and disables closed loop control if it was enabled.
  public void stop() {
    isRunningClosedLoop = false;
    m_motor.setVoltage(0);
    m_elevatorSim.setInput(0);
  }

  @Override
  public void periodic() {
    if (isHomed == false) {
      setVolts(Volts.of(-2));
      if (getLimitSwitch()) {
        isHomed = true;
        m_motor.setPosition(minHeight.in(Meters) / outputRatio);
      }
    }

    if (isRunningClosedLoop) {
      if (isHomed == false) {
        System.out.println("WARNING: Elevator not homed, but closed loop control is enabled");
        resetProfileState();
        return;
      }
      var next = profile.calculate(Constants.simulationTimestep.in(Seconds), setpoint, goalState);

      Voltage pidOutput = Volts.of(controller.calculate(getPosition().in(Meters), setpoint.position));

      Voltage feedforwardOutput = Volts.of(feedforward.calculateWithVelocities(setpoint.velocity, next.velocity));
      setVolts(pidOutput.plus(feedforwardOutput));

      setpoint = next;
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

  // resets the profile to the current state
  public void resetProfileState() {
    setpoint = new TrapezoidProfile.State(getPosition().in(Meters), getVelocity().in(MetersPerSecond));
  }

  public boolean isAtGoalPosition(Distance position) {
    return getPosition().isNear(position, toleranceOnReachedGoal);
  }

  // Runs the elevator to a specific position and then holds there.
  // If the command is interrupted the system will hold at the position it is currently at
  // Call stop() to disable all output power
  // public Command goToPositionCommand(Distance position) {
  //   return Commands.startRun(() -> {
  //     resetProfileState();
  //   }, () -> {
  //     setGoalAndEnableClosedLoop(position);
  //   }, this).until(() -> isAtGoalPosition(position)).handleInterrupt(() -> {
  //     System.out.println("WARNING: Elevator go to position command interrupted. Holding Current Position");
  //     goalState = new TrapezoidProfile.State(getPosition().in(Meters), 0);
  //   });
  // }

}
