package frc.robot.commands;

import java.util.Map;

import edu.wpi.first.networktables.EntryListenerFlags;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Climber2022;
import frc.robot.subsystems.Led2022;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Spitter2022;

import frc.robot.vision.BallTracking;
import frc.robot.vision.HubTarget;

public class Led2022UpdateCMD extends CommandBase {
    private final double TIMER_SPEED = 0.35;

    private Led2022 ledStrip;
    private LlamaNeck2022 llamaNeck = null;
    private Spitter2022 spitter = null;
    private Climber2022 climber = null;
    
    int color = 0;
    private Timer timer = new Timer();

    public static final double TARGET_MAX_RANGE = 3.0;
    public static final double TARGET_MAX_ANGLE = 4.0;
    public static final double BALL_MAX_RANGE = 3.0;
    public static final double BALL_MAX_ANGLE = 4.0;

    private COLORS_467 idleColorTop = COLORS_467.Blue;
    private COLORS_467 idleColorBottom = COLORS_467.Gold;
    private COLORS_467 hasBallColor = COLORS_467.White;
    private COLORS_467 seeTargetColor = COLORS_467.Gold;
    private COLORS_467 seeBallColor = COLORS_467.Blue;

    private NetworkTableEntry topFarLeft;
    private NetworkTableEntry topNearLeft;
    private NetworkTableEntry topFarRight;
    private NetworkTableEntry topNearRight;

    private NetworkTableEntry bottomFarLeft;
    private NetworkTableEntry bottomNearLeft;
    private NetworkTableEntry bottomFarRight;
    private NetworkTableEntry bottomNearRight;

    /*
     * Color blind preferred pallet includes White, Black, Red, Blue, Gold
     */
    public enum COLORS_467 {
        White(0xFF, 0xFF, 0xFF),
        Red(0xFF, 0x00, 0x00),
        Green(0x00, 0x80, 0x00),
        Blue(0x00, 0x00, 0xCC),
        Gold(0xFF, 0xC2, 0x0A),
        Pink(0xDC, 0x26, 0x7F),
        Black(0x00, 0x00, 0x00);

        public final int red;
        public final int green;
        public final int blue;

        COLORS_467(int red, int green, int blue) {
            this.red = red;
            this.green = green;
            this.blue = blue;
        }

    }

    public Led2022UpdateCMD(
        Led2022 ledStrip,
        Spitter2022 spitter, 
        LlamaNeck2022 llamaNeck, 
        Climber2022 climber) {

        this(ledStrip);
        
        this.spitter = spitter;
        this.llamaNeck = llamaNeck;

        addRequirements(spitter);
        addRequirements(llamaNeck);
        addRequirements(climber);
    }

    /**
     * For Testing, don't want to depend on other game pieces.
     * 
     * @param ledStrip the led strips
     */
    public Led2022UpdateCMD(Led2022 ledStrip) {
        this.ledStrip = ledStrip;
        addRequirements(ledStrip);

        if (DriverStation.getAlliance() == Alliance.Red) {
            seeBallColor = COLORS_467.Red;
        } else {
            seeBallColor = COLORS_467.Blue;
        }

        ShuffleboardTab tab = Shuffleboard.getTab("Operator");
        Shuffleboard.selectTab("Operator");
        ShuffleboardLayout layout = tab.getLayout("Grid Layout", "Grid Layout")
            .withPosition(0, 0) 
            .withSize(5, 2)
            .withProperties(Map.of("Label position", "HIDDEN"));
        
        topFarLeft = layout.add("TopFarLeft", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 0)
            .withSize(1,1)
            .withProperties(Map.of("colorWhenFalse", 0x000000))
            .withProperties(Map.of("colorWhenTrue", 0xE6E64D))
            .getEntry();

        topFarLeft.setBoolean(false);

        topNearLeft = layout.add("TopNearLeft", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(1, 0)
            .withSize(1,1)
            .withProperties(Map.of("colorWhenFalse", 0x000000))
            .withProperties(Map.of("colorWhenTrue", 0xE6E64D))
            .getEntry();

        topNearLeft.setBoolean(false);

        topNearRight = layout.add("TopNearRight", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(3, 0)
            .withSize(1,1)
            .withProperties(Map.of("colorWhenFalse", 0x000000))
            .withProperties(Map.of("colorWhenTrue", 0xFFC20A))
            .getEntry();

        topNearRight.setBoolean(false);

        topFarRight = layout.add("TopFarRight", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(4, 0)
            .withSize(1,1)
            .withProperties(Map.of("colorWhenFalse", 0x000000))
            .withProperties(Map.of("colorWhenTrue", 0xFFC20A))
            .getEntry();

            topFarRight.setBoolean(false);

            bottomFarLeft = layout.add("BottomFarLeft", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(0, 0)
            .withSize(1,1)
            .withProperties(Map.of("colorWhenFalse", 0x000000))
            .withProperties(Map.of("colorWhenTrue", 0xE6E64D))
            .getEntry();

        bottomFarLeft.setBoolean(false);

        bottomNearLeft = layout.add("BottomNearLeft", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(1, 0)
            .withSize(1,1)
            .withProperties(Map.of("colorWhenFalse", 0x000000))
            .withProperties(Map.of("colorWhenTrue", 0xE6E64D))
            .getEntry();

        bottomNearLeft.setBoolean(false);

        bottomNearRight = layout.add("BottomNearRight", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(3, 0)
            .withSize(1,1)
            .withProperties(Map.of("colorWhenFalse", 0x000000))
            .withProperties(Map.of("colorWhenTrue", 0xFFC20A))
            .getEntry();

        bottomNearRight.setBoolean(false);

        bottomFarRight = layout.add("BottomFarRight", false)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(4, 0)
            .withSize(1,1)
            .withProperties(Map.of("colorWhenFalse", 0x000000))
            .withProperties(Map.of("colorWhenTrue", 0xFFC20A))
            .getEntry();

            bottomFarRight.setBoolean(false);


        timer.start();

    }

    @Override
    public void initialize() {

        timer.reset();

    }

    @Override
    public void execute() { 

        if (DriverStation.isAutonomous() || DriverStation.isTeleop()) {
            idleColorTop = COLORS_467.Black;
            idleColorBottom = COLORS_467.Black;
        } else {
            idleColorTop = COLORS_467.Blue;
            idleColorBottom = COLORS_467.Gold;
        }

        boolean seesBall = BallTracking.hasBall();
        double ballDistance = BallTracking.getDistance();
        double ballAngle = BallTracking.getAngle();

        boolean seesTarget = HubTarget.hasTarget();
        double targetDistance = HubTarget.getDistance();
        double targetAngle = HubTarget.getAngle();

        if (seesTarget && targetDistance < TARGET_MAX_RANGE) {
            if  (Math.abs(targetAngle) < TARGET_MAX_ANGLE) {
                topFarLeft.setBoolean(false);
                topNearLeft.setBoolean(true);
                topNearRight.setBoolean(true);
                topFarRight.setBoolean(false);
            } else if (targetAngle < 0) {
                topFarLeft.setBoolean(true);
                topNearLeft.setBoolean(true);
                topNearRight.setBoolean(false);
                topFarRight.setBoolean(false);    
            } else {
                topFarLeft.setBoolean(false);
                topNearLeft.setBoolean(false);
                topNearRight.setBoolean(true);
                topFarRight.setBoolean(true);
            }
        } else {
            topFarLeft.setBoolean(false);
            topNearLeft.setBoolean(false);
            topNearRight.setBoolean(false);
            topFarRight.setBoolean(false);
        }

        if (seesBall && ballDistance < BALL_MAX_RANGE) {
            System.out.println("Sees red ball");
            if  (Math.abs(ballAngle) < BALL_MAX_ANGLE) {
                bottomFarLeft.setBoolean(false);
                bottomNearLeft.setBoolean(true);
                bottomNearRight.setBoolean(true);
                bottomFarRight.setBoolean(false);
            } else if (targetAngle < 0) {
                bottomFarLeft.setBoolean(true);
                bottomNearLeft.setBoolean(true);
                bottomNearRight.setBoolean(false);
                bottomFarRight.setBoolean(false);    
            } else {
                bottomFarLeft.setBoolean(false);
                bottomNearLeft.setBoolean(false);
                bottomNearRight.setBoolean(true);
                bottomFarRight.setBoolean(true);
            }
        } else {
            bottomFarLeft.setBoolean(false);
            bottomNearLeft.setBoolean(false);
            bottomNearRight.setBoolean(false);
            bottomFarRight.setBoolean(false);
        }

        if (climber != null && climber.isEnabled()) {
            setRainbowMovingUp();
//        } else if (spitter != null && spitter.isAtShootingSpeed()) {
//            //spitter.getCurrentCommand() instanceof Spitter2022ForwardCMD
//            setPurpleMovingUp();
        } else if (llamaNeck != null && llamaNeck.hasLowerBall()) {
            if (seesTarget && targetDistance < TARGET_MAX_RANGE &&  Math.abs(targetAngle) < TARGET_MAX_ANGLE) {
                setTop(seeTargetColor);
                setBottom(seeTargetColor);
            } else {
                set(hasBallColor);
            }
        } else if (llamaNeck != null && llamaNeck.hasUpperBall()) {
            setBottom(hasBallColor);
            if (seesBall && ballDistance < BALL_MAX_RANGE && Math.abs(ballAngle) < BALL_MAX_ANGLE) {
                set(seeBallColor);
            } else if (seesTarget && targetDistance < TARGET_MAX_RANGE &&  Math.abs(targetAngle) < TARGET_MAX_ANGLE) {
                setTop(seeTargetColor);
            } else {
                setTop(COLORS_467.Black); // Off
            }
        } else {
            if (seesBall && ballDistance < BALL_MAX_RANGE && Math.abs(ballAngle) < BALL_MAX_ANGLE) {
                set(seeBallColor);
            } else {
                setTop(idleColorTop);
                setBottom(idleColorBottom);
            }
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
        for (int i = RobotConstants.get().led2022LedCount()/2; i < RobotConstants.get().led2022LedCount(); i++) {
            ledStrip.setLED(i, color);
        }
    }

    public void setBottom(Color color) {
        for (int i = 0; i < RobotConstants.get().led2022LedCount()/2; i++) {
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
        if (timer.hasElapsed(TIMER_SPEED * (RobotConstants.get().led2022LedCount() + 1))) {
            timer.reset();
        }

        for (int i = 0; i < RobotConstants.get().led2022LedCount(); i++) {
            if (timer.hasElapsed(TIMER_SPEED * i)) {
                double timeUntilOff = Math.max(0, (TIMER_SPEED * (i + 1)) - timer.get());
                int brightness = (int) (255 * timeUntilOff);

                ledStrip.setRGB(i, 1 * brightness,0 * brightness, 1 * brightness);
             }
        }
    }

    public void setRainbowMovingUp() {
        if (timer.hasElapsed(TIMER_SPEED)) {
            color += 1;

            if (color > 360) color = 0;
            timer.reset();
        }
        
        for (int i = 0; i < RobotConstants.get().led2022LedCount(); i++) {
            ledStrip.setHSB(i, (color + (i * 360/RobotConstants.get().led2022LedCount())) % 360, 255, 127);
        }
    }

}
