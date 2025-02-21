package frc.robot.subsystems.autoalignment;

import java.io.File;
import java.io.IOException;
import java.time.Instant;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

import com.fasterxml.jackson.core.type.TypeReference;
import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.autoalignment.AutoAlignmentModel.ReefLocation;
import frc.robot.subsystems.autoalignment.AutoAlignmentModel.ReefOffset;

public class AutoAlignmentLoader {
    private Map<String, Pose2d> coralSproutPositions;
    private ReefLocation reefLocationBlue;
    private ReefLocation reefLocationRed;
    private Map<String, ReefOffset> reefOffsets = new LinkedHashMap<>();
    ShuffleboardTab tab = Shuffleboard.getTab("AutoAlignment");
    private GenericEntry xOffsetEntry, yOffsetEntry, outwardOffsetEntry, sideOffsetEntry;
    private SendableChooser<String> chooser;

    public AutoAlignmentLoader() {
        loadReefConfigs();
        generateCoralSproutPoses(0.0, 0.0, 0.0, 0.0);
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
    
            // Load reef offsets
            Map<String, ReefOffset> loadedOffsets = objectMapper.readValue(
                new File(Filesystem.getDeployDirectory(), AutoAlignmentConstants.REEF_OFFSETS_FILE),
                new TypeReference<>() {}
            );
            loadedOffsets.replaceAll((k, v) -> new ReefOffset(
                v.reefXPositionOffsetInches(),
                v.reefYPositionOffsetInches(),
                v.outwardTargetPositionOffsetInches(),
                v.sideTargetPositionOffsetInches()
            ));
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
                return t2.compareTo(t1);
            })
            .limit(20)
            .collect(LinkedHashMap::new, (map, entry) -> map.put(entry.getKey(), entry.getValue()), LinkedHashMap::putAll);
    }
    
    private Instant parseTimestamp(String timestamp) {
        try {
            if (!timestamp.endsWith("Z")) {
                timestamp += "Z";
            }
            return Instant.parse(timestamp);
        } catch (Exception e) {
            System.err.println("Invalid timestamp format: " + timestamp);
            return Instant.MIN;
        }
    }
    
    private ReefOffset getMostRecentReefOffset() {
        return reefOffsets.entrySet().stream()
            .findFirst()
            .map(Map.Entry::getValue)
            .orElse(new ReefOffset(0.0, 0.0, 0.0, 0.0));
    }
    
    private void generateCoralSproutPoses(double xOffset, double yOffset, double outwardOffset, double sideOffset) {
        ReefOffset reefOffset = getMostRecentReefOffset();
        double finalXOffset = xOffset != 0.0 ? xOffset : reefOffset.reefXPositionOffsetInches();
        double finalYOffset = yOffset != 0.0 ? yOffset : reefOffset.reefYPositionOffsetInches();
        double finalOutwardOffset = outwardOffset != 0.0 ? outwardOffset : reefOffset.outwardTargetPositionOffsetInches();
        double finalSideOffset = sideOffset != 0.0 ? sideOffset : reefOffset.sideTargetPositionOffsetInches();
    
        Map<String, Pose2d> blueSprouts = calculateCoralSproutPoses(
            reefLocationBlue.reefXPositionInches() + finalXOffset,
            reefLocationBlue.reefYPositionInches() + finalYOffset,
            finalOutwardOffset,
            finalSideOffset
        );
    
        Map<String, Pose2d> redSprouts = calculateCoralSproutPoses(
            reefLocationRed.reefXPositionInches() + finalXOffset,
            reefLocationRed.reefYPositionInches() + finalYOffset,
            finalOutwardOffset,
            finalSideOffset
        );
    
        Map<String, Pose2d> modifiedRedSprouts = new LinkedHashMap<>();
        for (Map.Entry<String, Pose2d> entry : redSprouts.entrySet()) {
            modifiedRedSprouts.put("R" + entry.getKey(), entry.getValue());
        }
    
        coralSproutPositions = new LinkedHashMap<>();
        coralSproutPositions.putAll(blueSprouts);
        coralSproutPositions.putAll(modifiedRedSprouts);
    }

    public Map<String, Pose2d> calculateCoralSproutPoses(double xPositionInches, double yPositionInches, 
                                                         double additionalOffsetInches, double sideOffsetInches) {
        final double R = 37.04;
        final double baseOffset = 11.66;
        final double[] panelAnglesDeg = {180, 240, 300, 0, 60, 120};
        final double[] panelAnglesDeg2 = {0, 60, 120, 180, 240, 300};
        final double halfSeparation = 6.47;
        final double[][] localSprouts = {
            {-halfSeparation, 0.0},
            {+halfSeparation, 0.0}
        };
        final String[] sproutNames = {"A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L"};

        Map<String, Pose2d> sproutPoses = new LinkedHashMap<>();
        int nameIndex = 0;

        for (int i = 0; i < panelAnglesDeg.length; i++) {
            double thetaDeg = panelAnglesDeg[i];
            double sproutRotationDeg = panelAnglesDeg2[i];
            
            double thetaRad = Math.toRadians(thetaDeg);
            Rotation2d panelRotation = new Rotation2d(Math.toRadians(sproutRotationDeg));

            double dx = Math.cos(thetaRad);
            double dy = Math.sin(thetaRad);

            double nx = -dy;
            double ny = dx;

            double panelCenterX = xPositionInches + R * dx;
            double panelCenterY = yPositionInches + R * dy;

            for (double[] offset : localSprouts) {
                double xLoc = offset[0];
                double yLoc = offset[1];

                double sproutX = panelCenterX + xLoc * nx + yLoc * dx;
                double sproutY = panelCenterY + xLoc * ny + yLoc * dy;

                double finalSproutX = sproutX + (baseOffset + additionalOffsetInches) * dx;
                double finalSproutY = sproutY + (baseOffset + additionalOffsetInches) * dy;

                finalSproutX += sideOffsetInches * nx;
                finalSproutY += sideOffsetInches * ny;

                Pose2d sproutPose = new Pose2d(
                    new Translation2d(Units.inchesToMeters(finalSproutX), Units.inchesToMeters(finalSproutY)),
                    panelRotation
                );

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
    
        // Use kTextView with properties to ensure editability
        xOffsetEntry = tab.add("X Offset", 0.0)
                          .withWidget(BuiltInWidgets.kTextView)
                          .getEntry();
        yOffsetEntry = tab.add("Y Offset", 0.0)
                          .withWidget(BuiltInWidgets.kTextView)
                          
                          .getEntry();
        outwardOffsetEntry = tab.add("Outward Offset", 0.0)
                                .withWidget(BuiltInWidgets.kTextView)
                                .getEntry();
        sideOffsetEntry = tab.add("Side Offset", 0.0)
                              .withWidget(BuiltInWidgets.kTextView)
                              .getEntry();
    
        tab.add("Apply Offsets", new InstantCommand(this::applyOffsets))
           .withWidget(BuiltInWidgets.kCommand);
    }

    public void update() {
        updateOffsetEntries();
    }
    
    private void updateOffsetEntries() {
        String selectedKey = chooser.getSelected();
        if (selectedKey != null && reefOffsets.containsKey(selectedKey)) {
            ReefOffset selectedOffset = reefOffsets.get(selectedKey);
    
            xOffsetEntry.setDouble(selectedOffset.reefXPositionOffsetInches());
            yOffsetEntry.setDouble(selectedOffset.reefYPositionOffsetInches());
            outwardOffsetEntry.setDouble(selectedOffset.outwardTargetPositionOffsetInches());
            sideOffsetEntry.setDouble(selectedOffset.sideTargetPositionOffsetInches());

            generateCoralSproutPoses(
                selectedOffset.reefXPositionOffsetInches(),
                selectedOffset.reefYPositionOffsetInches(),
                selectedOffset.outwardTargetPositionOffsetInches(),
                selectedOffset.sideTargetPositionOffsetInches()
            );
        }
    }
    
    private void applyOffsets() {
        // Flush NetworkTables to ensure latest values are available
        // NetworkTableInstance.getDefault().flush();
        double xOffset = xOffsetEntry.getDouble(0.0);
        double yOffset = yOffsetEntry.getDouble(0.0);
        double outwardOffset = outwardOffsetEntry.getDouble(0.0);
        double sideOffset = sideOffsetEntry.getDouble(0.0);

        System.out.println("Applying reef offsets: X=" + xOffset + ", Y=" + yOffset + 
                           ", Outward=" + outwardOffset + ", Side=" + sideOffset);

        ReefOffset newOffset = new ReefOffset(xOffset, yOffset, outwardOffset, sideOffset);
        String timestamp = Instant.now().toString();
        reefOffsets.put(timestamp, newOffset);

        ObjectMapper objectMapper = new ObjectMapper();
        try {
            objectMapper.writeValue(
                new File(Filesystem.getDeployDirectory(), AutoAlignmentConstants.REEF_OFFSETS_FILE),
                reefOffsets
            );
        } catch (IOException e) {
            e.printStackTrace();
        }

        String displayText = timestamp + " -> " + newOffset.toString();
        chooser.addOption(displayText, timestamp);
        chooser.setDefaultOption(displayText, timestamp);

        // // Update UI to reflect the applied values
        // xOffsetEntry.setDouble(xOffset);
        // yOffsetEntry.setDouble(yOffset);
        // outwardOffsetEntry.setDouble(outwardOffset);
        // sideOffsetEntry.setDouble(sideOffset);

        generateCoralSproutPoses(xOffset, yOffset, outwardOffset, sideOffset);
    }
    
    public Map<String, Pose2d> getCoralSproutPositions() {
        return Collections.unmodifiableMap(coralSproutPositions);
    }
}