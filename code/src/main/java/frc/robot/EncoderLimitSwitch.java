package frc.robot;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;

public class EncoderLimitSwitch {
    
    Supplier<Voltage> motorVolts;
    Supplier<AngularVelocity> encoderValue;

    Debouncer responsiveness = new Debouncer(Milliseconds.of(50).in(Seconds));

    void periodic () {
        
    }
}
