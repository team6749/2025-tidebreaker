package frc.robot.subsystems.autoalignment;

public class AutoAlignmentModel {
    public record ReefOffset(
        double reefXPositionOffsetInches, 
        double reefYPositionOffsetInches, 
        double outwardTargetPositionOffsetInches,
        double sideTargetPositionOffsetInches) { // New field for side offset
        
        // Override toString for better display in Shuffleboard
        @Override
        public String toString() {
            return String.format("X: %.2f, Y: %.2f, Outward: %.2f, Side: %.2f",
                reefXPositionOffsetInches, reefYPositionOffsetInches,
                outwardTargetPositionOffsetInches, sideTargetPositionOffsetInches);
        }
    }

    public record ReefLocation(double reefXPositionInches, double reefYPositionInches) {}
}