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
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DutyCycleEncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.swerve.SwerveConstants;

public class Arm extends SubsystemBase {
  boolean closedLoop = false;

  public static Angle simStartAngle = Radians.of(0);
  PIDController armPID = new PIDController(0, 0, 0);
  ArmFeedforward feedForward = new ArmFeedforward(0, 0,0);
  TalonFX armMotor = new TalonFX(Constants.armMotorPort);//todo put in actual motor
  DCMotor m_armGearbox = DCMotor.getFalcon500(Constants.armMotorPort);
  
  public static Distance armLength = Meters.of(0.4);
  public static Mass armMass = Kilograms.of(1);
  public static Angle tolerance = Radians.of(0.08);

DutyCycleEncoder encoder = new DutyCycleEncoder(2); // I don't think we have one on the arm but, for the sake of the simulation, let's try it.

  Rotation2d angle = Rotation2d.kZero;
  Angle maxAngle = Radians.of(Math.PI);
  Angle minAngle = Radians.of(0);
  AngularVelocity maxVelocity = RadiansPerSecond.of(0);
  AngularAcceleration maxAcceleration = RadiansPerSecondPerSecond.of(0); //to do, find these values.
  private final TrapezoidProfile trapezoidProfile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(maxVelocity.in(RadiansPerSecond),
        maxAcceleration.in(RadiansPerSecondPerSecond)));

    private final SingleJointedArmSim m_armSim = new SingleJointedArmSim(
        m_armGearbox,
        1 / Constants.armGearRatio,
        SingleJointedArmSim.estimateMOI(armLength.in(Meters), armMass.in(Kilograms)),
        armLength.in(Meters),
        minAngle.in(Radians),
        maxAngle.in(Radians),
        true,
        simStartAngle.in(Radians),0,0 // Add noise with a std-dev of 1 tick. Add std-dev with 2 arguments otherwise it breaks.
    );
    private final DutyCycleEncoderSim encoderSim = new DutyCycleEncoderSim(encoder);
    //Create a Mechanism2d display of an Arm with a fixed ArmTower and moving Arm.
    private final Mechanism2d mech2d = new Mechanism2d(2, 2);
    private final MechanismRoot2d armPivot = mech2d.getRoot("ArmPivot", 1, 1);
    @SuppressWarnings("unused")
    private final MechanismLigament2d armTower = armPivot
        .append(new MechanismLigament2d(
          "ArmTower", 
          armLength.in(Meters), 
          -90, 
          2, 
          new Color8Bit(Color.kAqua)
          )
          );

        private final MechanismLigament2d armMech = armPivot.append(
        new MechanismLigament2d(
            "Arm",
            30,
            simStartAngle.in(Degrees),
            6,
            new Color8Bit(Color.kYellow)));

    TrapezoidProfile.State desiredState = new State(getPosition().in(Radians),0);
    TrapezoidProfile.State currentState;

  /** Creates a new Arm. */
  public Arm() {}

   public void simulationPeriodic() {
     //if (encoderSim != null) {
       m_armSim.update(Constants.simulationTimestep.in(Seconds));
       encoderSim.set(Radians.of(m_armSim.getAngleRads()).in(Rotations) * Constants.armGearRatio);
  // } else {
  //     System.out.println("Simulated encoder is not initialized.");
  //   }
   }

  public AngularVelocity getVelocity() {
    return RadiansPerSecond.of(armMotor.getVelocity().getValueAsDouble() * Constants.armGearRatio * Math.PI * 2);
  }

  @Override
  public void periodic() {
    if(closedLoop) {
      Rotation2d currentAngle = getAngle();
      TrapezoidProfile.State currentState = new State(currentAngle.getRadians(),getVelocity().in(RadiansPerSecond));
      var nextState = trapezoidProfile.calculate(Constants.simulationTimestep.in(Seconds), currentState, desiredState);
      Voltage output = Volts.of(armPID.calculate(currentState.position,nextState.position));
      Voltage feedForwardSystem = Volts.of(feedForward.calculate(currentState.velocity,nextState.velocity));
      if(RobotBase.isSimulation()) {
        m_armSim.setInputVoltage(output.in(Volts) + feedForwardSystem.in(Volts));
      }
      else {
        armMotor.setVoltage(output.in(Volts) + feedForwardSystem.in(Volts)); //porbably should do a seperate implementation here but this works for now
      }
    } else { //open loop
      if(RobotBase.isSimulation()) {
        m_armSim.setInputVoltage(1);
      }
      else {
        armMotor.setVoltage(1); 
      }
    }
    angle = Rotation2d.fromRotations(armMotor.getPosition().getValueAsDouble()/ Constants.armGearRatio);
    armMech.setAngle(getPosition().in(Radians));
    
    // This method will be called once per scheduler run
  }

  public void runClosedLoop(Rotation2d desiredAngle) {
    desiredState = new State(desiredAngle.getRadians(),0);
    closedLoop = true;
  }

  public Command armCommand(Rotation2d desiredAngle) {
    Command command = Commands.run(() ->{
      runClosedLoop(desiredAngle);
    }, this);
    return command;
  }

  public void neutralState() {
    armMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public Rotation2d getAngle() {
      return angle;
  }

  public Angle getPosition() {
    return Radians.of(armMotor.getPosition().getValueAsDouble() * Constants.armGearRatio * Math.PI * 2);
  }

  public Boolean isAtTarget(Angle position) {
    return getPosition().isNear(position, tolerance);
  }

  public void resetProfileState() {
    currentState = new TrapezoidProfile.State(getPosition().in(Rotations), getVelocity().in(RadiansPerSecond));
  }

  public Command goToPositionArm(Angle desiredAngle) {
    Rotation2d goalAngle = Rotation2d.fromRadians(desiredAngle.in(Radians));
    Command command = Commands.startRun(() -> {
      resetProfileState();
    },
      () -> {runClosedLoop(goalAngle);},
      this
    ).until(() -> isAtTarget(desiredAngle)).handleInterrupt(() -> {
      System.out.println("WARNING: Arm go to position command interrupted. Holding Current Position");
      desiredState = new TrapezoidProfile.State(getPosition().in(Rotations), 0);
    });
    return command;
}
}
  

