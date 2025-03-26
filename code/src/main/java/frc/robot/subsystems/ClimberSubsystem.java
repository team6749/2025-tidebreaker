package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

@Logged
public class ClimberSubsystem extends SubsystemBase {
  Boolean isClimbing = false;
  Timer timer = new Timer();
  TalonFX climberMotor = new TalonFX(Constants.climberMotorPort);
  Voltage inputVoltage = Volts.of(2);

  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    timer.start();
    climberMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command climbCommand() {
    isClimbing = true;
    return Commands.runEnd(() -> {
      climberMotor.setVoltage(inputVoltage.in(Volts));
    }, () -> {
      stop();
    }, this);
  }

  public Command unclimbCommand() {
    return Commands.runEnd(() -> {
      climberMotor.setVoltage(-inputVoltage.in(Volts));
    }, () -> {
      stop();
    }, this);
  }

  public Boolean isClimbing() {
    return isClimbing;
  }

  public void stop() {
    isClimbing = false;
    climberMotor.setVoltage(0);
  }
}
