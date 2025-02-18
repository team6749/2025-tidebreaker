package frc.robot.subsystems.autoalignment;


public class Model {
    public record ReefOffset(double reefXPositionOffsetInches, 
        double reefYPositionOffsetInches, 
        double outwardTargetPositionOffsetInches) {}

    public record ReefLocation(double reefXPositionInches, double reefYPositionInches) {}
}

