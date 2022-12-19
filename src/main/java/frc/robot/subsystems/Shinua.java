// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorSensorV3.ColorSensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ColorSensorResolution;
import com.revrobotics.ColorSensorV3.GainFactor;

import edu.wpi.first.math.Pair;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.robot.Constants;

public class Shinua extends SubsystemBase {

    // private final TalonFX inputMotor, outputMotor;
    private final ColorSensorV3 colorSensor;
    private ColorMatch colorMatcher;
    private ColorSensorResolution resolution;
    private ColorSensorMeasurementRate measurementRate;
    private GainFactor gainFactor;

    private Color red;
    private Color blue;
    private double minConfidence;

    private final SendableChooser<ColorSensorResolution> resolutionChooser;
    private final SendableChooser<ColorSensorMeasurementRate> measurementRateChooser;
    private final SendableChooser<GainFactor> gainFactorChooser;

    public Shinua() {
        // inputMotor = new TalonFX(Constants.INPUT_MOTOR_ID);
        // outputMotor = new TalonFX(Constants.OUTPUT_MOTOR_ID);
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        colorMatcher = new ColorMatch();

        resolutionChooser = new SendableChooser<>();
        measurementRateChooser = new SendableChooser<>();
        gainFactorChooser = new SendableChooser<>();

        init();
    }

    public void init() {
        configureDevices();
        configureDashboard();
    }

    public void configureDevices() {
        // inputMotor.configFactoryDefault();
        // outputMotor.configFactoryDefault();
    }

    public void configureDashboard() {
        resolutionChooser.setDefaultOption("13-bit", ColorSensorResolution.kColorSensorRes13bit);
        resolutionChooser.addOption("16-bit", ColorSensorResolution.kColorSensorRes16bit);
        resolutionChooser.addOption("17-bit", ColorSensorResolution.kColorSensorRes17bit);
        resolutionChooser.addOption("18-bit", ColorSensorResolution.kColorSensorRes18bit);
        resolutionChooser.addOption("19-bit", ColorSensorResolution.kColorSensorRes19bit);
        resolutionChooser.addOption("20-bit", ColorSensorResolution.kColorSensorRes20bit);
        SmartDashboard.putData("Color Sensor Resolution", resolutionChooser);

        measurementRateChooser.setDefaultOption("50ms", ColorSensorMeasurementRate.kColorRate50ms);
        measurementRateChooser.addOption("500ms", ColorSensorMeasurementRate.kColorRate500ms);
        measurementRateChooser.addOption("25ms", ColorSensorMeasurementRate.kColorRate25ms);
        measurementRateChooser.addOption("200ms", ColorSensorMeasurementRate.kColorRate200ms);
        measurementRateChooser.addOption("2000ms", ColorSensorMeasurementRate.kColorRate2000ms);
        measurementRateChooser.addOption("100ms", ColorSensorMeasurementRate.kColorRate100ms);
        measurementRateChooser.addOption("1000ms", ColorSensorMeasurementRate.kColorRate1000ms);
        SmartDashboard.putData("Color Sensor Measurement Rate", measurementRateChooser);

        gainFactorChooser.setDefaultOption("1x", GainFactor.kGain1x);
        gainFactorChooser.addOption("3x", GainFactor.kGain3x);
        gainFactorChooser.addOption("6x", GainFactor.kGain6x);
        gainFactorChooser.addOption("9x", GainFactor.kGain9x);
        gainFactorChooser.addOption("18x", GainFactor.kGain18x);
        SmartDashboard.putData("Color Sensor Gain Factor", gainFactorChooser);

        SmartDashboard.putNumber("Current Red R", 0);
        SmartDashboard.putNumber("Current Red G", 0);
        SmartDashboard.putNumber("Current Red B", 0);

        SmartDashboard.putNumber("Current Blue R", 0);
        SmartDashboard.putNumber("Current Blue G", 0);
        SmartDashboard.putNumber("Current Blue B", 0);

        SmartDashboard.putNumber("Minimum Confidence", 0);
    }

    @Override
    public void periodic() {
        var newResolution = resolutionChooser.getSelected();
        var newMeasurementRate = measurementRateChooser.getSelected();
        var newGainFactor = gainFactorChooser.getSelected();

        if (newResolution != resolution || newMeasurementRate != measurementRate
                || newGainFactor != gainFactor) {
            resolution = newResolution;
            measurementRate = newMeasurementRate;
            gainFactor = newGainFactor;
            colorSensor.configureColorSensor(resolution, measurementRate, gainFactor);
        }

        double redR = SmartDashboard.getNumber("Current Red R", 0);
        double redG = SmartDashboard.getNumber("Current Red G", 0);
        double redB = SmartDashboard.getNumber("Current Red B", 0);

        if (redR != red.red || redG != red.green || redB != red.blue) {
            red = new Color(redR, redG, redB);
            colorMatcher = new ColorMatch();
            colorMatcher.addColorMatch(red);
            colorMatcher.addColorMatch(blue);
        }

        double blueR = SmartDashboard.getNumber("Current Blue R", 0);
        double blueG = SmartDashboard.getNumber("Current Blue G", 0);
        double blueB = SmartDashboard.getNumber("Current Blue B", 0);

        if (blueR != blue.red || blueG != blue.green || blueB != blue.blue) {
            blue = new Color(blueR, blueG, blueB);
            colorMatcher = new ColorMatch();
            colorMatcher.addColorMatch(red);
            colorMatcher.addColorMatch(blue);
        }

        double newMinConfidence = SmartDashboard.getNumber("Minimum Confidence", 0);
        
        if (newMinConfidence != minConfidence) {
            minConfidence = newMinConfidence;
            colorMatcher.setConfidenceThreshold(1 - minConfidence);
        }
    }

    public Pair<BallColor, Double> getColor() {
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

        return new Pair<>(fromColor(match.color), match.confidence);
    }

    public enum BallColor {
        RED, BLUE, UNKNOWN;

        public String toString() {
            switch (this) {
                case RED:
                    return "Red";
                case BLUE:
                    return "Blue";
                default:
                    return "Unknown";
            }
        }

        public static BallColor fromString(String colorString) {
            switch (colorString) {
                case "Red":
                    return RED;
                case "Blue":
                    return BLUE;
                default:
                    return UNKNOWN;
            }
        }
    }

    public BallColor fromColor(Color color) {
        if (color.equals(red)) {
            return BallColor.RED;
        } else if (color.equals(blue)) {
            return BallColor.BLUE;
        } else {
            return BallColor.UNKNOWN;
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Red", colorSensor::getRed, null);
        builder.addDoubleProperty("Green", colorSensor::getGreen, null);
        builder.addDoubleProperty("Blue", colorSensor::getBlue, null);

        builder.addStringProperty("Guessed Color", () -> {return getColor().getFirst().toString();}, null);
        builder.addDoubleProperty("Confidence", () -> {return getColor().getSecond();}, null);

        builder.addDoubleProperty("Proximity", colorSensor::getProximity, null);
    }
}
