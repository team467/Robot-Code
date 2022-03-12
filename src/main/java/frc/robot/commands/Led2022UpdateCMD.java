package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Climber2022;
import frc.robot.subsystems.Indexer2022;
import frc.robot.subsystems.Led2022;
import frc.robot.subsystems.LlamaNeck2022;

import frc.robot.vision.BallTracking;
import frc.robot.vision.HubTarget;

public class Led2022UpdateCMD extends CommandBase {

    public static final boolean USE_BATTERY_CHECK = true;
    public static final double BATTER_MIN_VOLTAGE = 9.0;

    private final double SHOOTING_TIMER_SPEED = 0.02;
    private final double RAINBOW_TIMER_SPEED = 0.02;
    private final int RAINBOW_AMOUNT = 10;

    private Led2022 ledStrip;
    private LlamaNeck2022 llamaNeck = null;
    private Indexer2022 indexer = null;
    private Climber2022 climber = null;
    
    int color = 0;
    private Timer rainbowTimer = new Timer();
    private Timer purpleTimer = new Timer();

    public static final double TARGET_MAX_RANGE = 100.0;
    public static final double TARGET_MAX_ANGLE = 15.0;
    public static final double BALL_MAX_RANGE = 100.0;
    public static final double BALL_MAX_ANGLE = 15.0;

    private COLORS_467 hasBallColor = COLORS_467.White;
    private COLORS_467 seeTargetColor = COLORS_467.Gold;
    private COLORS_467 seeBallColor = COLORS_467.Blue;
    private COLORS_467 batteryCheckColor = COLORS_467.Orange;

    private SuppliedValueWidget<Boolean> targetIndicatorWidget[];
    private SuppliedValueWidget<Boolean> seeBallIndicatorWidget[];
    private SuppliedValueWidget<Boolean> hasBallIndicatorWidget[];

    private NetworkTableEntry[] targetIndicators;
    private NetworkTableEntry[] seeBallIndicators;
    private NetworkTableEntry[] hasBallIndicators;

    private static final int TARGET_INDICATOR_OFFSET_X = 7;
    private static final int TARGET_INDICATOR_OFFSET_Y = 0;
    private static final int SEE_BALL_INDICATOR_OFFSET_X = 7;
    private static final int SEE_BALL_INDICATOR_OFFSET_Y = 1;
    private static final int HAVE_BALL_INDICATOR_OFFSET_X = 8;
    private static final int HAVE_BALL_INDICATOR_OFFSET_Y = 2;

    /*
     * Color blind preferred pallet includes White, Black, Red, Blue, Gold
     */
    public enum COLORS_467 {
        White(0xFF, 0xFF, 0xFF, 0xdc267f00),
        Red(0xFF, 0x00, 0x00, 0x99000000),
        Green(0x00, 0x80, 0x00, 0x33663300),
        Blue(0x00, 0x00, 0xCC, 0x1a339900),
        Gold(0xFF, 0xC2, 0x0A, 0xe6e64d00),
        Pink(0xDC, 0x26, 0x7F, 0xdc267f00),
        Orange(0xFE, 0x61, 0x00, 0xfe6100),
        Black(0x00, 0x00, 0x00, 0x00000000);

        public final int red;
        public final int green;
        public final int blue;
        public final int shuffleboard;

        COLORS_467(int red, int green, int blue, int shuffleboard) {
            this.red = red;
            this.green = green;
            this.blue = blue;
            this.shuffleboard = shuffleboard;
        }

    }

    public Led2022UpdateCMD(
        Led2022 ledStrip,
        Indexer2022 indexer, 
        LlamaNeck2022 llamaNeck, 
        Climber2022 climber) {

        this(ledStrip);
        
        this.indexer = indexer;
        this.llamaNeck = llamaNeck;
        this.climber = climber;

    }

    /**
     * For Testing, don't want to depend on other game pieces.
     * 
     * @param ledStrip the led strips
     */
    public Led2022UpdateCMD(Led2022 ledStrip) {
        this.ledStrip = ledStrip;
        addRequirements(ledStrip);

        ShuffleboardTab tab = Shuffleboard.getTab("Operator");
        Shuffleboard.selectTab("Operator");

        if (CameraServer.getServer("Driver Front Camera") != null) {
            tab.add("Driver Front Camera", CameraServer.getServer("Driver Front Camera"))
            .withPosition(0, 0)
            .withSize(3, 3);
        }
        
        if (CameraServer.getServer("Driver Back Camera") != null) {
            tab.add("Driver Back Camera", CameraServer.getServer("Driver Back Camera"))
            .withPosition(3, 0)
            .withSize(3, 3);
        }

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("Indicators");
        NetworkTable widgetTable = inst.getTable("Operator Widgets");

        targetIndicators = new NetworkTableEntry[4];
        seeBallIndicators = new NetworkTableEntry[4];
        hasBallIndicators = new NetworkTableEntry[2];

        for (int i = 0; i < 4; i++) {
            targetIndicators[i] = table.getEntry("Target "+i);
            targetIndicators[i].setBoolean(false);
            final String name = "Target " +i;
            targetIndicatorWidget[i] = tab.addBoolean(name + " Widget", () -> {
                return NetworkTableInstance.getDefault()
                    .getTable("Indicators")
                    .getSubTable("Target")
                    .getEntry(name)
                    .getBoolean(false);
            })
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(TARGET_INDICATOR_OFFSET_X + i, TARGET_INDICATOR_OFFSET_Y)
            .withSize(1,1)
            .withProperties(Map.of(
                "Color when false", COLORS_467.Black.shuffleboard,
                "Color when true", seeTargetColor.shuffleboard));
            targetIndicatorWidget[i].buildInto(widgetTable, table);
        }

        //     targetIndicators[i] = tab.add("Target " + i, false)
        //         .withWidget(BuiltInWidgets.kBooleanBox)
        //         .withPosition(TARGET_INDICATOR_OFFSET_X + i, TARGET_INDICATOR_OFFSET_Y)
        //         .withSize(1,1)
        //         .withProperties(Map.of(
        //             "Color when false", COLORS_467.Black.shuffleboard,
        //             "Color when true", seeTargetColor.shuffleboard))
        //         .getEntry();
        //     targetIndicators[i].setBoolean(false);
        // }
        
        for (int i = 0; i < 4; i++) {
            seeBallIndicators[i] = table.getEntry("See Ball " + i);
            seeBallIndicators[i].setBoolean(false);
            final String name = "See Ball " + i;
            seeBallIndicatorWidget[i] = tab.addBoolean(name + " Widget", () -> {
                return NetworkTableInstance.getDefault()
                    .getTable("Indicators")
                    .getSubTable("See Ball")
                    .getEntry(name)
                    .getBoolean(false);
            })
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(SEE_BALL_INDICATOR_OFFSET_X + i, SEE_BALL_INDICATOR_OFFSET_Y)
            .withSize(1,1)
            .withProperties(Map.of(
                "Color when false", COLORS_467.Black.shuffleboard,
                "Color when true", seeBallColor.shuffleboard));
            seeBallIndicatorWidget[i].buildInto(widgetTable, table);

            // seeBallIndicators[i] = tab.add("See Ball " + i, false)
            // .withWidget(BuiltInWidgets.kBooleanBox)
            // .withPosition(SEE_BALL_INDICATOR_OFFSET_X + i, SEE_BALL_INDICATOR_OFFSET_Y)
            // .withSize(1,1)
            // .withProperties(Map.of(
            //     "Color when false", COLORS_467.Black.shuffleboard,
            //     "Color when true", seeBallColor.shuffleboard))
            // .getEntry();
            // seeBallIndicators[i].setBoolean(false);
        }

        for (int i = 0; i < 2; i++) {
            hasBallIndicators[i] = table.getEntry("Has Ball " + i);
            hasBallIndicators[i].setBoolean(false);
            final String name = "Has Ball " + i;
            hasBallIndicatorWidget[i] = tab.addBoolean(name + " Widget", () -> {
                return NetworkTableInstance.getDefault()
                    .getTable("Indicators")
                    .getSubTable("Has Ball")
                    .getEntry(name)
                    .getBoolean(false);
            })
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(HAVE_BALL_INDICATOR_OFFSET_X + i, HAVE_BALL_INDICATOR_OFFSET_Y)
            .withSize(1,1)
            .withProperties(Map.of(
                "Color when false", COLORS_467.Black.shuffleboard,
                "Color when true", hasBallColor.shuffleboard));
            hasBallIndicatorWidget[i].buildInto(widgetTable, table);
    

            // hasBallIndicators[i] = tab.add("Has Ball " + i, false)
            // .withWidget(BuiltInWidgets.kBooleanBox)
            // .withPosition(HAVE_BALL_INDICATOR_OFFSET_X + i, HAVE_BALL_INDICATOR_OFFSET_Y)
            // .withSize(1,1)
            // .withProperties(Map.of(
            //     "Color when false", COLORS_467.Black.shuffleboard,
            //     "Color when true", hasBallColor.shuffleboard))
            // .getEntry();
            // hasBallIndicators[i].setBoolean(false);
        }

        rainbowTimer.start();
        purpleTimer.start();
    }

    private void indicators(
        NetworkTableEntry[] indicators, boolean isValid, 
        double distance, double range,
        double angle, double maxAngle) {
        if (isValid && distance < range) {
            if  (Math.abs(angle) < maxAngle) {
                indicators[0].setBoolean(false);
                indicators[1].setBoolean(true);
                indicators[2].setBoolean(true);
                indicators[3].setBoolean(false);
            } else if (angle < 0) {
                indicators[0].setBoolean(true);
                indicators[1].setBoolean(true);
                indicators[2].setBoolean(false);
                indicators[3].setBoolean(false);    
            } else {
                indicators[0].setBoolean(false);
                indicators[1].setBoolean(false);
                indicators[2].setBoolean(true);
                indicators[3].setBoolean(true);
            }
        } else {
            indicators[0].setBoolean(false);
            indicators[1].setBoolean(false);
            indicators[2].setBoolean(false);
            indicators[3].setBoolean(false);
        }
    }

    private void cargoIndicator(
        NetworkTableEntry[] indicators, 
        int capacity, int amount) {
        for (int i = 0; i < amount; i++) {
            indicators[capacity-1-i].setBoolean(true);
        }
        for (int i = amount; i < capacity; i++) {
            indicators[capacity-1-i].setBoolean(true);
        }
    }

    @Override
    public void initialize() {
        rainbowTimer.reset();
        purpleTimer.reset();
    }

    @Override
    public void execute() { 
        if (DriverStation.getAlliance() == Alliance.Red) {
            seeBallColor = COLORS_467.Red;
        } else {
            seeBallColor = COLORS_467.Blue;
        }

        if (DriverStation.isEnabled()) {

            boolean seesTarget = HubTarget.hasTarget();
            double targetDistance = HubTarget.getDistance();
            double targetAngle = HubTarget.getAngle();
            indicators(targetIndicators, seesTarget, 
                targetDistance, TARGET_MAX_RANGE,
                targetAngle, TARGET_MAX_ANGLE);

            boolean seesBall = BallTracking.hasBall();
            double ballDistance = BallTracking.getDistance();
            double ballAngle = BallTracking.getAngle();
            indicators(seeBallIndicators, seesBall, 
                ballDistance, BALL_MAX_RANGE,
                ballAngle, BALL_MAX_ANGLE);
            
            if (USE_BATTERY_CHECK && RobotController.getBatteryVoltage() <= BATTER_MIN_VOLTAGE) {
                set(batteryCheckColor);
            } else if (climber != null && climber.isEnabled()) {
                if (climber.getCurrentCommand() instanceof Climber2022UpCMD) {
                    setRainbowMovingUp();
                } else if (climber.getCurrentCommand() instanceof Climber2022DownCMD) {
                    setRainbowMovingDown();
                } else {
                    setRainbow();
                } 
            } else if (indexer != null && indexer.isShooting()) {
                setPurpleMovingUp();
            } else if (llamaNeck != null && llamaNeck.hasLowerBall()) {
                cargoIndicator(hasBallIndicators, 2, 2);
                if (seesTarget && targetDistance < TARGET_MAX_RANGE &&  Math.abs(targetAngle) < TARGET_MAX_ANGLE) {
                    setTop(seeTargetColor);
                    setBottom(seeTargetColor);
                } else {
                    set(hasBallColor);
                }
            } else if (llamaNeck != null && llamaNeck.hasUpperBall()) {
                cargoIndicator(hasBallIndicators, 2, 1);
                setBottom(hasBallColor);
                if (seesBall && ballDistance < BALL_MAX_RANGE && Math.abs(ballAngle) < BALL_MAX_ANGLE) {
                    set(seeBallColor);
                } else if (seesTarget && targetDistance < TARGET_MAX_RANGE &&  Math.abs(targetAngle) < TARGET_MAX_ANGLE) {
                    setTop(seeTargetColor);
                } else {
                    setTop(COLORS_467.Black); // Off
                }
            } else {
                cargoIndicator(hasBallIndicators, 2, 0);
                if (seesBall && ballDistance < BALL_MAX_RANGE && Math.abs(ballAngle) < BALL_MAX_ANGLE) {
                    set(seeBallColor);
                } else {
                    setTop(COLORS_467.Black);
                    setBottom(COLORS_467.Black);
                }
            }
        } else {
            setTop(COLORS_467.Blue);
            setBottom(COLORS_467.Gold);
        }

        ledStrip.sendData();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void set(Color color) {
        setTop(color);
        setBottom(color);
    }

    public void setTop(Color color) {
        for (int i = 0; i < RobotConstants.get().led2022LedCount()/2; i++) {
            ledStrip.setLED(i, color);
        }
    }

    public void setBottom(Color color) {
        for (int i = RobotConstants.get().led2022LedCount()/2; i < RobotConstants.get().led2022LedCount(); i++) {
            ledStrip.setLED(i, color);
        }
    }

    public void set(COLORS_467 color) {
        setTop(color);
        setBottom(color);
    }

    public void setTop(COLORS_467 color) {
        for (int i = RobotConstants.get().led2022LedCount()/2; i < RobotConstants.get().led2022LedCount(); i++) {
            ledStrip.setRGB(i, color.red, color.green, color.blue);
        }
    }

    public void setBottom(COLORS_467 color) {
        for (int i = 0; i < RobotConstants.get().led2022LedCount()/2; i++) {
            ledStrip.setRGB(i, color.red, color.green, color.blue);
        }
    }

    public void setPurpleMovingUp() {
        if (purpleTimer.hasElapsed(SHOOTING_TIMER_SPEED * (RobotConstants.get().led2022LedCount() + 1))) {
            purpleTimer.reset();
        }

        for (int i = 0; i < RobotConstants.get().led2022LedCount(); i++) {
            if (purpleTimer.hasElapsed(SHOOTING_TIMER_SPEED * i)) {
                double timeUntilOff = Math.max(0, (SHOOTING_TIMER_SPEED * (i + 1)) - purpleTimer.get());
                int brightness = (int) (255 * timeUntilOff);

                ledStrip.setRGB(i, 1 * brightness,0 * brightness, 1 * brightness);
             }
        }
    }

    public void setRainbowMovingUp() {
        if (rainbowTimer.hasElapsed(RAINBOW_TIMER_SPEED)) {
            color -= RAINBOW_AMOUNT;

            if (color > 360) color = 0;
            rainbowTimer.reset();
        }
        
        for (int i = 0; i < RobotConstants.get().led2022LedCount(); i++) {
            ledStrip.setHSB(i, (color + (i * 360/RobotConstants.get().led2022LedCount())) % 360, 255, 127);
        }
    }

    public void setRainbowMovingDown() {
        if (rainbowTimer.hasElapsed(RAINBOW_TIMER_SPEED)) {
            color += RAINBOW_AMOUNT;

            if (color < 0) color = 360;
            rainbowTimer.reset();
        }
        
        for (int i = 0; i < RobotConstants.get().led2022LedCount(); i++) {
            ledStrip.setHSB(i, (color + (i * 360/RobotConstants.get().led2022LedCount())) % 360, 255, 127);
        }
    }

    public void setRainbow() {
        rainbowTimer.reset();
        for (int i = 0; i < RobotConstants.get().led2022LedCount(); i++) {
            ledStrip.setHSB(i, (color + (i * 360/RobotConstants.get().led2022LedCount())) % 360, 255, 127);
        }
    }

}
