package frc.robot.subsystems.autoalignment;

import java.io.File;
import java.io.IOException;
import java.time.Instant;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.autoalignment.AutoAlignmentModel.ReefLocation;
import frc.robot.subsystems.autoalignment.AutoAlignmentModel.ReefOffset;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

public class AutoAlignmentLoader {
    private Map<String, Pose2d> coralSproutPositions;
    private ReefLocation reefLocationBlue;
    private ReefLocation reefLocationRed;
    private Map<String, ReefOffset> reefOffsets = new LinkedHashMap<>(); // Stores recent 20 entries
    ShuffleboardTab tab = Shuffleboard.getTab("AutoAlignment");
    private GenericEntry xOffsetEntry, yOffsetEntry, outwardOffsetEntry;
    private SendableChooser<String> chooser;

    public AutoAlignmentLoader() {
        loadReefConfigs();
        generateCoralSproutPoses(0.0, 0.0, 0.0);
        setupShuffleboard();
    }

    private void loadReefConfigs() {
        ObjectMapper objectMapper = new ObjectMapper();
        try {
            Map<String, Object> reefLocations = objectMapper.readValue(
                new File(Filesystem.getDeployDirectory(), AutoAlignmentConstants.REEF_LOCATIONS_FILE),
                new TypeReference<>() {}
            );
            @SuppressWarnings("unchecked")
            Map<String, Object> reefLocationCalculations = (Map<String, Object>) reefLocations.get("reefLocationCalulations");
    
            // Load blue reef data
            @SuppressWarnings("unchecked")
            Map<String, Object> blueData = (Map<String, Object>) reefLocationCalculations.get("blue");
            @SuppressWarnings("unchecked")
            Map<String, Object> blueXData = (Map<String, Object>) blueData.get("x");
            @SuppressWarnings("unchecked")
            Map<String, Object> blueYData = (Map<String, Object>) blueData.get("y");
            double blueX = ((Number) blueXData.get("locationInches")).doubleValue();
            double blueY = ((Number) blueYData.get("locationInches")).doubleValue();
            reefLocationBlue = new ReefLocation(blueX, blueY);
    
            // Load red reef data
            @SuppressWarnings("unchecked")
            Map<String, Object> redData = (Map<String, Object>) reefLocationCalculations.get("red");
            @SuppressWarnings("unchecked")
            Map<String, Object> redXData = (Map<String, Object>) redData.get("x");
            @SuppressWarnings("unchecked")
            Map<String, Object> redYData = (Map<String, Object>) redData.get("y");
            double redX = ((Number) redXData.get("locationInches")).doubleValue();
            double redY = ((Number) redYData.get("locationInches")).doubleValue();
            reefLocationRed = new ReefLocation(redX, redY);
    
            // Optionally, set a default reef location
            // reefLocation = reefLocationBlue;
    
            // Load reef offsets as before
            Map<String, ReefOffset> loadedOffsets = objectMapper.readValue(
                new File(Filesystem.getDeployDirectory(), AutoAlignmentConstants.REEF_OFFSETS_FILE),
                new TypeReference<>() {}
            );
            reefOffsets = sortAndLimitOffsets(loadedOffsets);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
    
    private LinkedHashMap<String, ReefOffset> sortAndLimitOffsets(Map<String, ReefOffset> offsets) {
        return offsets.entrySet().stream()
            .sorted((e1, e2) -> {
                Instant t1 = parseTimestamp(e1.getKey());
                Instant t2 = parseTimestamp(e2.getKey());
                return t2.compareTo(t1); // Sort by most recent first
            })
            .limit(20) // Keep only the 20 most recent
            .collect(LinkedHashMap::new, (map, entry) -> map.put(entry.getKey(), entry.getValue()), LinkedHashMap::putAll);
    }
    
    private Instant parseTimestamp(String timestamp) {
        try {
            if (!timestamp.endsWith("Z")) {
                timestamp += "Z"; // Ensure it's a valid ISO 8601 timestamp
            }
            return Instant.parse(timestamp);
        } catch (Exception e) {
            System.err.println("Invalid timestamp format: " + timestamp);
            return Instant.MIN; // If invalid, push it to the end of sorting
        }
    }
    
    private ReefOffset getMostRecentReefOffset() {
        return reefOffsets.entrySet().stream()
            .findFirst() // The first entry is now the most recent due to sorting
            .map(Map.Entry::getValue)
            .orElse(new ReefOffset(0.0, 0.0, 0.0)); // Default if no valid offsets exist
    }
    

    private void generateCoralSproutPoses(double xOffset, double yOffset, double outwardOffset) {
        ReefOffset reefOffset = getMostRecentReefOffset();
        double finalXOffset = xOffset != 0.0 ? xOffset : reefOffset.reefXPositionOffsetInches();
        double finalYOffset = yOffset != 0.0 ? yOffset : reefOffset.reefYPositionOffsetInches();
        double finalOutwardOffset = outwardOffset != 0.0 ? outwardOffset : reefOffset.outwardTargetPositionOffsetInches();
    
        // Generate blue reef sprouts (12 positions)
        Map<String, Pose2d> blueSprouts = calculateCoralSproutPoses(
            reefLocationBlue.reefXPositionInches() + finalXOffset,
            reefLocationBlue.reefYPositionInches() + finalYOffset,
            finalOutwardOffset
        );
    
        // Generate red reef sprouts (12 positions)
        Map<String, Pose2d> redSprouts = calculateCoralSproutPoses(
            reefLocationRed.reefXPositionInches() + finalXOffset,
            reefLocationRed.reefYPositionInches() + finalYOffset,
            finalOutwardOffset
        );
    
        // Prefix red reef keys with "R" to distinguish them
        Map<String, Pose2d> modifiedRedSprouts = new LinkedHashMap<>();
        for (Map.Entry<String, Pose2d> entry : redSprouts.entrySet()) {
            modifiedRedSprouts.put("R" + entry.getKey(), entry.getValue());
        }
    
        // Combine both sets into one map (total 24 positions)
        coralSproutPositions = new LinkedHashMap<>();
        coralSproutPositions.putAll(blueSprouts);
        coralSproutPositions.putAll(modifiedRedSprouts);
    }

    public Map<String, Pose2d> calculateCoralSproutPoses(double xPositionInches, double yPositionInches, double additionalOffsetInches) {
        // 1) Reef center and panel radius
        final double R  = 37.04;  // distance from reef center to the panel's midpoint
        final double baseOffset = 11.66; // Existing sprout projection

        // 2) Panel angles (in degrees), going COUNTERCLOCKWISE around the hex.
        final double[] panelAnglesDeg = {180, 240, 300, 0, 60, 120};
        final double[] panelAnglesDeg2 = {0, 60, 120, 180, 240, 300};
        
        // 3) Two sprouts per panel, each offset ±6.47" perpendicular to the radial line.
        final double halfSeparation = 6.47;
        final double[][] localSprouts = {
            {-halfSeparation, 0.0},   // Sprout #1
            {+halfSeparation, 0.0}    // Sprout #2
        };

        // 4) Sprout names in the order they are generated (counterclockwise).
        final String[] sproutNames = {
            "A","B",  "C","D",  "E","F",
            "G","H",  "I","J",  "K","L"
        };

        // We'll store them in a LinkedHashMap to preserve insertion order
        Map<String, Pose2d> sproutPoses = new LinkedHashMap<>();
        int nameIndex = 0;

        // 5) Calculate final positions (and orientation) for each panel
        for (int i = 0; i < panelAnglesDeg.length; i++) {
            double thetaDeg = panelAnglesDeg[i];
            double sproutRotationDeg = panelAnglesDeg2[i];
            
            // Panel rotation in radians
            double thetaRad = Math.toRadians(thetaDeg);
            Rotation2d panelRotation = new Rotation2d(Math.toRadians(sproutRotationDeg));

            // Radial direction (unit vector)
            double dx = Math.cos(thetaRad);
            double dy = Math.sin(thetaRad);

            // Perpendicular (tangential) direction
            double nx = -dy;
            double ny = dx;

            // Panel center in field coords
            double panelCenterX = xPositionInches + R * dx;
            double panelCenterY = yPositionInches + R * dy;

            // Two sprouts per panel
            for (double[] offset : localSprouts) {
                double xLoc = offset[0];  // tangential offset
                double yLoc = offset[1];  // radial offset

                // Rotate + translate local offsets to field coordinates
                double sproutX = panelCenterX + xLoc * nx + yLoc * dx;
                double sproutY = panelCenterY + xLoc * ny + yLoc * dy;

                // 🔹 Offset the target position outward along the panel vector
                double finalSproutX = sproutX + (baseOffset + additionalOffsetInches) * dx;
                double finalSproutY = sproutY + (baseOffset + additionalOffsetInches) * dy;

                Pose2d sproutPose = new Pose2d(
                    new Translation2d(Units.inchesToMeters(finalSproutX), Units.inchesToMeters(finalSproutY)),
                    panelRotation
                );

                // Store with name A..L
                sproutPoses.put(sproutNames[nameIndex], sproutPose);
                nameIndex++;
            }
        }

        return sproutPoses;
    }

    private void setupShuffleboard() {
        chooser = new SendableChooser<>();
        for (Map.Entry<String, ReefOffset> entry : reefOffsets.entrySet()) {
            String displayText = entry.getKey() + " -> " + entry.getValue();
            chooser.addOption(displayText, entry.getKey());
        }
        chooser.setDefaultOption("Select Reef Offset", reefOffsets.keySet().iterator().next());
    
        tab.add("Reef Offsets", chooser)
           .withWidget(BuiltInWidgets.kComboBoxChooser);
    
        // Offset Inputs (Text Fields)
        xOffsetEntry = tab.add("X Offset", 0.0)
                          .withWidget(BuiltInWidgets.kTextView)
                          .getEntry();
        yOffsetEntry = tab.add("Y Offset", 0.0)
                          .withWidget(BuiltInWidgets.kTextView)
                          .getEntry();
        outwardOffsetEntry = tab.add("Outward Offset", 0.0)
                                .withWidget(BuiltInWidgets.kTextView)
                                .getEntry();
    
        tab.add("Apply Offsets", new InstantCommand(this::applyOffsets))
           .withWidget(BuiltInWidgets.kCommand);
    }

    // New public method to trigger UI updates
    public void update() {
        updateOffsetEntries();
    }
    
    private void updateOffsetEntries() {
        String selectedKey = chooser.getSelected();
        if (selectedKey != null && reefOffsets.containsKey(selectedKey)) {
            ReefOffset selectedOffset = reefOffsets.get(selectedKey);
    
            // Update UI values
            xOffsetEntry.setDouble(selectedOffset.reefXPositionOffsetInches());
            yOffsetEntry.setDouble(selectedOffset.reefYPositionOffsetInches());
            outwardOffsetEntry.setDouble(selectedOffset.outwardTargetPositionOffsetInches());

            generateCoralSproutPoses(selectedOffset.reefXPositionOffsetInches(), selectedOffset.reefYPositionOffsetInches(), selectedOffset.outwardTargetPositionOffsetInches());
        }
    }
    
    private void applyOffsets() {
        // Read latest values
        double xOffset = xOffsetEntry.getDouble(0.0);
        double yOffset = yOffsetEntry.getDouble(0.0);
        double outwardOffset = outwardOffsetEntry.getDouble(0.0);
    
        System.out.println("Read Offsets: X=" + xOffset + " Y=" + yOffset + " Outward=" + outwardOffset);
    
        // Create new offset and store in the map
        ReefOffset newOffset = new ReefOffset(xOffset, yOffset, outwardOffset);
        String timestamp = Instant.now().toString();
        reefOffsets.put(timestamp, newOffset);
    
        // Save updated reefOffsets to JSON
        ObjectMapper objectMapper = new ObjectMapper();
        try {
            objectMapper.writeValue(
                new File(Filesystem.getDeployDirectory(), AutoAlignmentConstants.REEF_OFFSETS_FILE),
                reefOffsets
            );
        } catch (IOException e) {
            e.printStackTrace();
        }
    
        // Update the chooser with new selection
        String displayText = timestamp + " -> " + newOffset.toString();
        chooser.addOption(displayText, timestamp);
        chooser.setDefaultOption(displayText, timestamp);
    
        System.out.println("Applying Offsets: X=" + xOffset + " Y=" + yOffset + " Outward=" + outwardOffset);
    
        // pass in optional values
        generateCoralSproutPoses(xOffset, yOffset, outwardOffset);
    }
    
    public Map<String, Pose2d> getCoralSproutPositions() {
        return Collections.unmodifiableMap(coralSproutPositions);
    }
}
