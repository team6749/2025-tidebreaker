package frc.robot.loggers;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.epilogue.CustomLoggerFor;
import edu.wpi.first.epilogue.logging.ClassSpecificLogger;
import edu.wpi.first.epilogue.logging.EpilogueBackend;

@CustomLoggerFor(TalonFX.class)
public class TalonFXLogger extends ClassSpecificLogger<TalonFX> {

    public TalonFXLogger() {
        super(TalonFX.class);
    }

    @Override
    public void update(EpilogueBackend backend, TalonFX motor) {
        backend.log("Output", motor.get());
        backend.log("Stator Current", motor.getStatorCurrent().getValue().in(Amps));
        backend.log("Temp", motor.getDeviceTemp().getValue().in(Celsius));
        backend.log("Temp Processor", motor.getProcessorTemp().getValue().in(Celsius));
    }
}