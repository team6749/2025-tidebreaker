// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

// NOTE THIS ARM DOES NOT SUPPORT ROLLOVER.
// THERE IS NO DETECTION FOR ROLLOVER ON THE
// ENCODER, PID CONTROLLER, FEEDFORWARD, OR MOTION PROFILE
// THIS MEANS THE ENCODER ZERO POSITION CANNOT BE WITHIN
// THE TRAVEL RANGE.
@Logged
public class Arm extends SubsystemBase {

  Alert encoderDisconnectedAlert = new Alert("Arm Encoder Disconnected", AlertType.kError);

  // Constants
  public static Distance armLength = Meters.of(0.4);
  public static Mass armMass = Kilograms.of(5);
  public static int armReduction = 25 * 2; // 15 for gearbox, 2 for the sproket
  public static double outputRatio = (1.0 / armReduction);
  // Offset to apply to the encoder to make the arm have a zero position parallel
  // to the floor
  public static Angle angleOffset = Degrees.of(0);
  public static Angle minAngle = Degrees.of(-90);
  public static Angle maxAngle = Degrees.of(90);
  public static Angle simStartAngle = Degrees.of(0);
  public static Angle toleranceOnReachedGoal = Degrees.of(2);
  public static AngularVelocity maxVelocity = DegreesPerSecond.of(60);
  public static AngularAcceleration maxAccerleration = DegreesPerSecondPerSecond.of(60);

  private final DCMotor m_armGearbox = DCMotor.getFalcon500(1);

  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
  private final PIDController controller = new PIDController(12, 0, 0);
  // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/feedforward.html#armfeedforward
  private final ArmFeedforward feedforward = new ArmFeedforward(
      0,
      0.6, // Most important value
      0,
      0);

  // TODO convert from using a trapezoidal profile to an ExponentialProfile sysid
  private final TrapezoidProfile profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxVelocity.in(RadiansPerSecond),
          maxAccerleration.in(RadiansPerSecondPerSecond)));

  // Intermediate goal position, used for closed loop control.
  private TrapezoidProfile.State setpoint = new TrapezoidProfile.State(0, 0);
  private TrapezoidProfile.State goalState = setpoint;
  private boolean isRunningClosedLoop = false;

  private final TalonFX m_motor = new TalonFX(1);
  private final DutyCycleEncoder encoder = new DutyCycleEncoder(2);
  private boolean encoderConnected = true;

  private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
      m_armGearbox,
      armReduction,
      SingleJointedArmSim.estimateMOI(armLength.in(Meters), armMass.in(Kilograms)),
      armLength.in(Meters),
      minAngle.in(Radians),
      maxAngle.in(Radians),
      true,
      0,
      simStartAngle.in(Radians),
      0.0 // Add noise with a std-dev of 1 tick
  );

  // Simulation Control
  private final DutyCycleEncoderSim m_encoderSim = new DutyCycleEncoderSim(encoder);

  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d m_mech2d = new Mechanism2d(2, 2);
  private final MechanismRoot2d m_armPivot = m_mech2d.getRoot("ArmPivot", 1, 1);
  @SuppressWarnings("unused")
  private final MechanismLigament2d m_armTower = m_armPivot
      .append(new MechanismLigament2d("ArmTower", armLength.in(Meters), -90, 2, new Color8Bit(Color.kAqua)));
  private final MechanismLigament2d m_arm = m_armPivot.append(
      new MechanismLigament2d(
          "Arm",
          30,
          simStartAngle.in(Degrees),
          6,
          new Color8Bit(Color.kYellow)));

  // Returns angles where zero is parallel to the floor (gravity). Values can be
  // negative
  public Angle getPosition() {
    // The duty cycle encoder does not handle rollover
    // https://docs.wpilib.org/en/stable/docs/software/hardware-apis/sensors/encoders-software.html#duty-cycle-encoders-the-dutycycleencoder-class
    return Rotations.of(encoder.get() * outputRatio).plus(angleOffset);
  }

  public AngularVelocity getVelocity() {
    // TODO implement
    return RotationsPerSecond.of(0);
  }

  /** Creates a new ArmSubsystem. */
  public Arm() {
    SmartDashboard.putData("Arm Sim", m_mech2d);
  }

  public void setGoalAndEnableClosedLoop(Angle goal) {
    isRunningClosedLoop = true;
    goalState = new TrapezoidProfile.State(goal.in(Radians), 0);
  }

  public void runOpenLoop(Voltage volts) {
    isRunningClosedLoop = false;
    setVolts(volts);
  }

  private void setVolts(Voltage volts) {
    m_motor.setVoltage(volts.in(Volts));
    if (Robot.isSimulation()) {
      m_armSim.setInputVoltage(volts.in(Volts));
    }
  }

  // Stops motors and disables closed loop control if it was enabled.
  public void stop() {
    isRunningClosedLoop = false;
    m_motor.setVoltage(0);
    m_armSim.setInputVoltage(0);
  }

  @Override
  public void periodic() {
    encoderConnected = encoder.isConnected();
    encoderDisconnectedAlert.set(encoderConnected == false);

    if (isRunningClosedLoop) {
      if (encoderConnected == false) {
        System.out.println("WARNING: Arm encoder disconnected");
        // Resetting state to avoid sudden movement when reconnected
        resetProfileState();
        stop();
        return;
      }
      var currentPosition = getPosition();
      var next = profile.calculate(Constants.simulationTimestep.in(Seconds), setpoint, goalState);

      Voltage pidOutput = Volts.of(controller.calculate(currentPosition.in(Radians), setpoint.position));
      Voltage feedForwardOutput = Volts.of(feedforward.calculate(setpoint.position, setpoint.velocity));
      // For some reason this feed forward breaks.. not sure why. but it just makes
      // everything nan after 1 iteration
      // Voltage feedForwardOutput =
      // Volts.of(feedforward.calculateWithVelocities(currentPosition.in(Radians),
      // setpoint.velocity,
      // next.velocity));

      setVolts(pidOutput.plus(feedForwardOutput));
      setpoint = next;
    }

    // Telemetry
    m_arm.setAngle(getPosition().in(Degrees));
  }

  @Override
  public void simulationPeriodic() {
    // Next, we update it. The standard loop time is 20ms.
    m_armSim.update(Constants.simulationTimestep.in(Seconds));
    m_encoderSim.set(Radians.of(m_armSim.getAngleRads()).in(Rotations) / outputRatio);
  }

  // resets the profile to the current state
  public void resetProfileState() {
    setpoint = new TrapezoidProfile.State(getPosition().in(Radians), getVelocity().in(RadiansPerSecond));
  }

  public boolean isAtGoalPosition(Angle position) {
    return getPosition().isNear(position, toleranceOnReachedGoal);
  }

  // Runs the elevator to a specific position and then holds there.
  // If the command is interrupted the system will hold at the position it is currently at
  // Call stop() to disable all output power
  public Command goToPositionCommand(Angle position) {
    return Commands.startRun(() -> {
      resetProfileState();
    }, () -> {
      setGoalAndEnableClosedLoop(position);
    }, this).until(() -> isAtGoalPosition(position)).handleInterrupt(() -> {
      System.out.println("WARNING: Arm go to position command interrupted. Holding Current Position");
      goalState = new TrapezoidProfile.State(getPosition().in(Rotations), 0);
    });
  }
}
