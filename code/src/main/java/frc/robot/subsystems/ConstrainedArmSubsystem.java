// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
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

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Robot;

@Logged
public class ConstrainedArmSubsystem extends SubsystemBase {
  private Alert encoderDisconnectedAlert = new Alert("Arm Encoder Disconnected", AlertType.kError);

  public static Angle simStartAngle = Degrees.of(-90);
  public static Angle angleOffset = Rotations.of(RobotBase.isSimulation() ? 0 : -0.565); //-0.144 the encoder value - 0.25 for standard position.
  public static Distance armLength = Meters.of(0.2);
  public static Mass armMass = Kilograms.of(0.3);
  public static Angle tolerance = Degrees.of(2.5);
  public static Angle maxAngle = Degrees.of(90);
  public static Angle minAngle = Degrees.of(-90);

  private PIDController armPID = new PIDController(2, 0, 0);
  private ArmFeedforward feedForward = new ArmFeedforward(0, 0.2, 0.78);
  private TalonFX armMotor = new TalonFX(Constants.armMotorID);
  private DCMotor m_armGearbox = DCMotor.getFalcon500(1);

  private boolean closedLoop = false;
  private boolean encoderConnected = true;

  DutyCycleEncoder encoder = new DutyCycleEncoder(2);
 
  AngularVelocity maxVelocity = DegreesPerSecond.of(240);
  AngularAcceleration maxAcceleration = DegreesPerSecondPerSecond.of(720);

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

  TrapezoidProfile.State targetState = new State(0, 0);
  TrapezoidProfile.State setpointState = new State(0, 0);

  private Angle lastPosition = simStartAngle;
  private AngularVelocity lastVelocity = DegreesPerSecond.zero();

    // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutAngle m_angle = Radians.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutAngularVelocity m_velocity = RadiansPerSecond.mutable(0);

  // Create a new SysId routine for characterizing the shooter.
  private final SysIdRoutine m_sysIdRoutine =
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              voltage -> {
              setVolts(voltage);
              },
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("arm")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            armMotor.getMotorVoltage().getValueAsDouble(), Volts))
                    .angularPosition(m_angle.mut_replace(getPosition().in(Rotations), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(getVelocity().in(RotationsPerSecond), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));

  /** Creates a new Arm. */
  public ConstrainedArmSubsystem() {
    stop();
    SmartDashboard.putData("Arm Sim", mech2d);


    var angleMotorCurrentLimits = new CurrentLimitsConfigs().withStatorCurrentLimit(Amps.of(60))
        .withStatorCurrentLimitEnable(true);
    armMotor.getConfigurator().apply(angleMotorCurrentLimits);
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

    Angle nextPosition = getPosition();
    lastVelocity = nextPosition.minus(lastPosition).div(Constants.simulationTimestep);
    lastPosition = nextPosition;

    if (closedLoop) {
      if (encoderConnected == false) {
        System.out.println("WARNING: Arm encoder disconnected");
        // Resetting state to avoid sudden movement when reconnected
        resetProfileState();
        stop();
        return;
      }
      var currentAngle = getPosition();
      var nextState = trapezoidProfile.calculate(Constants.simulationTimestep.in(Seconds), setpointState, targetState);

      Voltage pidoutput = Volts.of(armPID.calculate(currentAngle.in(Radians), setpointState.position));
      Voltage feedForwardOutput = Volts.of(feedForward.calculate(setpointState.position, setpointState.velocity));

      setVolts(pidoutput.plus(feedForwardOutput));
      setpointState = nextState;
    }
    armMech.setAngle(getPosition().in(Degrees));

    // This method will be called once per scheduler run
  }

  private void runClosedLoop(Angle desiredAngle) {
    targetState = new TrapezoidProfile.State(desiredAngle.in(Radians), 0);
    closedLoop = true;
  }

  public void neutralState() {
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public AngularVelocity getVelocity() {
    return lastVelocity;
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
    setpointState = new TrapezoidProfile.State(getPosition().in(Radians), getVelocity().in(RadiansPerSecond));
  }

   public Command goToPositionCommand(Angle desiredAngle) {
    if(desiredAngle.gt(maxAngle)) {
      Exception exception =  new Exception("WARNING: Arm instructed to go more than max angle. " + desiredAngle.toString());
      System.out.println(exception.getMessage());
      exception.printStackTrace();
      if(Robot.isSimulation()) {
        // TODO: Crash the simulator
      }
      return goToPositionCommandPrivate(maxAngle);
    }
    if(desiredAngle.lt(minAngle)) {
      Exception exception =  new Exception("WARNING: Arm instructed to go less than min angle. " + desiredAngle.toString());
      System.out.println(exception.getMessage());
      exception.printStackTrace();
      if(Robot.isSimulation()) {
        // TODO: Crash the simulator
      }
      return goToPositionCommandPrivate(minAngle);
    }
    return goToPositionCommandPrivate(desiredAngle);
  }


  private Command goToPositionCommandPrivate(Angle desiredAngle) {
    return Commands.startRun(() -> {
      resetProfileState();
    }, () -> {
      runClosedLoop(desiredAngle);
    }, this).until(() -> isAtTarget(desiredAngle)).handleInterrupt(() -> {
      System.out.println("WARNING: Arm go to position command interrupted. Holding Current Position");
      targetState = new TrapezoidProfile.State(getPosition().in(Radians), 0);
    });
  }

  private void setVolts(Voltage volts) {
    armMotor.setVoltage(volts.in(Volts));
    if (RobotBase.isSimulation()) {
      simArm.setInputVoltage(volts.in(Volts));
    }
  }

  public void runVolts(Voltage volts) {
    setVolts(volts);
    closedLoop = false;
  }

  public Command runVoltsCommand(Voltage volts) {
    return Commands.runEnd(() -> runVolts(volts), () -> stop(), this);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.quasistatic(direction).until(() -> direction == SysIdRoutine.Direction.kForward ? getPosition().in(Radians) > maxAngle.in(Radians) - 0.2 : getPosition().in(Radians) < minAngle.in(Radians) + 0.2);
  };

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return m_sysIdRoutine.dynamic(direction).until(() -> direction == SysIdRoutine.Direction.kForward ? getPosition().in(Radians) > maxAngle.in(Radians) - 0.2 : getPosition().in(Radians) < minAngle.in(Radians) + 0.2);
  }
}
