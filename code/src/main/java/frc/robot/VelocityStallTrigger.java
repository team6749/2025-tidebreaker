package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Provides triggers for detecting motor velocity stalls based on expected and measured velocity.
 * 
 * <p>This class offers two static methods to create {@link Trigger} objects that activate when 
 * a motor experiences a velocity-based stall for a specified duration.
 * 
 * <p>Features:
 * <ul>
 *   <li>Detects stalls when the motor's velocity is below a threshold for a set delay</li>
 *   <li>Ignores stalls when the motor is not instructed to move</li>
 *   <li>Uses a {@link Debouncer} to filter transient velocity fluctuations</li>
 *   <li>Supports directional stall detection using {@link TriggerDirection}</li>
 * </ul>
 * 
 * <p>Methods:
 * <ul>
 *   <li>{@link #fromZero(TriggerDirection, AngularVelocity, Supplier, Supplier, Time)} - 
 *       Triggers when the velocity remains near zero beyond a threshold</li>
 *   <li>{@link #fromMeasurement(TriggerDirection, AngularVelocity, Supplier, Supplier, Time)} - 
 *       Triggers when measured velocity deviates from expected velocity for too long</li>
 * </ul>
 * 
 * This utility helps monitor motor stalls without requiring physical limit switches.
 */
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
