package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// Can be used to trigger when there is a velocity based stall.
//
public class VelocityStallTrigger {

    // Triggers when the motor is below the velocity for the stallDelay.
    // The motor is assumed to be free when it is not instructed to move
    // This is good for acting like a limit switch
    static Trigger fromZero (TriggerDirection triggerDirection,
    AngularVelocity stalledVelocity, Supplier<AngularVelocity> motorExpected, Supplier<AngularVelocity> measurement, Time stallDelay) {
        Debouncer stallDebounce = new Debouncer(stallDelay.in(Seconds));
        return new Trigger(() -> {
            double motorMagnitude = motorExpected.get().baseUnitMagnitude();
            // The motor is not considered stalled if it is not supposed to be moving
            boolean isImmediatelyStalled = motorMagnitude != 0 && measurement.get().isNear(RadiansPerSecond.zero(), stalledVelocity);
            boolean isStalled = stallDebounce.calculate(isImmediatelyStalled);

            if(isStalled) {
                // Check if we meet the stall direction condiditon
                if(triggerDirection == TriggerDirection.Both) {
                    return true;
                }
                if(motorMagnitude > 0 && triggerDirection == TriggerDirection.Positive) {
                    return true;
                }
                if(motorMagnitude < 0 && triggerDirection == TriggerDirection.Negative) {
                    return true;
                }
            }

            return false;
        });
    }

    // Triggers when the measurement is further away than the stalled velocity for a given time.
    static Trigger fromMeasurement (TriggerDirection triggerDirection,
    AngularVelocity stalledVelocity, Supplier<AngularVelocity> motorExpected, Supplier<AngularVelocity> measurement, Time stallDelay) {
        Debouncer stallDebounce = new Debouncer(stallDelay.in(Seconds));
        return new Trigger(() -> {
            AngularVelocity motorExpectedValue = motorExpected.get();
            // The motor is not considered stalled if it is not supposed to be moving
            boolean isImmediatelyStalled = measurement.get().isNear(motorExpectedValue, stalledVelocity) == false;
            boolean isStalled = stallDebounce.calculate(isImmediatelyStalled);

            if(isStalled) {
                // Check if we meet the stall direction condiditon
                if(triggerDirection == TriggerDirection.Both) {
                    return true;
                }
                if(motorExpectedValue.magnitude() > 0 && triggerDirection == TriggerDirection.Positive) {
                    return true;
                }
                if(motorExpectedValue.magnitude() < 0 && triggerDirection == TriggerDirection.Negative) {
                    return true;
                }
            }

            return false;
        });
    }
}
