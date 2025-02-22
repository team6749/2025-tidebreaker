// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
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
import frc.robot.subsystems.swerve.SwerveConstants;

@Logged
public class Arm extends SubsystemBase {
  Alert encoderDisconnectedAlert = new Alert("Arm Encoder Disconnected", AlertType.kError);

  boolean closedLoop = false;
  boolean encoderConnected = true;
  boolean motorInverted = true;

  public static Angle simStartAngle = Degrees.of(-90);
  public static Angle angleOffset = Radians.of(RobotBase.isSimulation() ? 0:(- Math.PI / 2));
  PIDController armPID = new PIDController(15, 0, 0);
  ArmFeedforward feedForward = new ArmFeedforward(0, 0.1, 2);
  TalonFX armMotor = new TalonFX(Constants.armMotorPort);// todo put in actual motor
  DCMotor m_armGearbox = DCMotor.getFalcon500(1);

  public static Distance armLength = Meters.of(0.2);
  public static Mass armMass = Kilograms.of(0.3);
  public static Angle tolerance = Radians.of(0.04);

  DutyCycleEncoder encoder = new DutyCycleEncoder(2); // I don't think we have one on the arm but, for the sake of the
                                                      // simulation, let's try it.

  Angle maxAngle = Degrees.of(90);
  Angle minAngle = Degrees.of(-90);
  AngularVelocity maxVelocity = RadiansPerSecond.of(1);
  AngularAcceleration maxAcceleration = RadiansPerSecondPerSecond.of(1.5); // to do, find these values.
  private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(maxVelocity.in(RadiansPerSecond),
          maxAcceleration.in(RadiansPerSecondPerSecond)));

  private final SingleJointedArmSim simArm = new SingleJointedArmSim(
      m_armGearbox,
      1 / Constants.armGearRatio,
      SingleJointedArmSim.estimateMOI(armLength.in(Meters), armMass.in(Kilograms)),
      armLength.in(Meters),
      minAngle.in(Radians),
      maxAngle.in(Radians),
      true,
      simStartAngle.in(Radians), 0, 0 // Add noise with a std-dev of 1 tick. Add std-dev with 2 arguments otherwise it
                                      // breaks.
  );
  private final DutyCycleEncoderSim encoderSim = new DutyCycleEncoderSim(encoder);
  // Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
  private final Mechanism2d mech2d = new Mechanism2d(2, 2);
  private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 1, 1);
  @SuppressWarnings("unused")
  private final MechanismLigament2d armTower = armPivot
      .append(new MechanismLigament2d(
          "ArmTower",
          armLength.in(Meters),
          -90,
          2,
          new Color8Bit(Color.kAqua)));

  private final MechanismLigament2d armMech = armPivot.append(
      new MechanismLigament2d(
          "Arm",
          0.5,
          simStartAngle.in(Degrees),
          3,
          new Color8Bit(Color.kYellow)));

  TrapezoidProfile.State desiredState = new State(0, 0);
  TrapezoidProfile.State currentState = new State(0, 0);

  /** Creates a new Arm. */
  public Arm() {
    SmartDashboard.putData("Arm Sim", mech2d);
  }

  public void simulationPeriodic() {
    simArm.update(Constants.simulationTimestep.in(Seconds));
    encoderSim.set(Radians.of(simArm.getAngleRads()).in(Rotations));
  }

  @Override
  public void periodic() {
    simArm.getOutput();
    encoderConnected = encoder.isConnected();
    encoderDisconnectedAlert.set(encoderConnected == false);

    if (closedLoop) {
      if (encoderConnected == false) {
        System.out.println("WARNING: Arm encoder disconnected");
        // Resetting state to avoid sudden movement when reconnected
        resetProfileState();
        stop();
        return;
      }
      var currentAngle = getPosition();
      var nextState = trapezoidProfile.calculate(Constants.simulationTimestep.in(Seconds), currentState, desiredState);

      Voltage pidoutput = Volts.of(armPID.calculate(currentAngle.in(Radians), currentState.position));
      Voltage feedForwardOutput = Volts.of(feedForward.calculate(currentState.position, currentState.velocity));

      setVolts(pidoutput.plus(feedForwardOutput));
      currentState = nextState;
    }
    armMech.setAngle(getPosition().in(Degrees));

    // This method will be called once per scheduler run
  }

  public void runClosedLoop(Angle desiredAngle) {
    desiredState = new TrapezoidProfile.State(desiredAngle.in(Radians), 0);
    closedLoop = true;
  }

  public void neutralState() {
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public AngularVelocity getVelocity() {
    // TODO implement
    return RotationsPerSecond.of(0);
  }

  public Angle getPosition() {
    return Rotations.of(encoder.get()).plus(angleOffset);
  }

  public Boolean isAtTarget(Angle position) {
    return getPosition().isNear(position, tolerance);
  }

  public void stop() {
    armMotor.setVoltage(0.1);
    simArm.setInputVoltage(0);
    closedLoop = false;
  }

  public void resetProfileState() {
    currentState = new TrapezoidProfile.State(getPosition().in(Radians), getVelocity().in(RadiansPerSecond));
  }

  public Command goToPositionArm(Angle desiredAngle) {
    return Commands.startRun(() -> {
      resetProfileState();
    }, () -> {
      runClosedLoop(desiredAngle);
    }, this).until(() -> isAtTarget(desiredAngle)).handleInterrupt(() -> {
      System.out.println("WARNING: Arm go to position command interrupted. Holding Current Position");
      desiredState = new TrapezoidProfile.State(getPosition().in(Radians), 0);
    });
  }

  private void setVolts(Voltage volts) {
    armMotor.setVoltage(volts.in(Volts)); // porbably should do a seperate implementation here but this works for now
    if (RobotBase.isSimulation()) {
      simArm.setInputVoltage(volts.in(Volts));
    }
  }

  public void runVolts(Voltage volts) {
    setVolts(volts);
    closedLoop = false;
  }

  public Command runOpenLoopCommand(Voltage Volts) {
    return Commands.runEnd(() -> runVolts(Volts), () -> stop(), this);
  }
}
