package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.Climber2022;
import frc.robot.subsystems.Dashboard2022;
import frc.robot.subsystems.Led2022;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Shooter2022;
import frc.robot.vision.BallTracking;
import frc.robot.vision.HubTarget;

public class Led2022UpdateCMD extends CommandBase {

    public static final boolean USE_BATTERY_CHECK = true;
    public static final double BATTER_MIN_VOLTAGE = 9.0;

    private final double SHOOTING_TIMER_SPEED = 0.1;
    private final double RAINBOW_TIMER_SPEED = 0.02;
    private final int RAINBOW_AMOUNT = 10;

    private Led2022 ledStrip;
    private Dashboard2022 dashboard;
    private LlamaNeck2022 llamaNeck = null;
    private Shooter2022 shooter = null;
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
        LlamaNeck2022 llamaNeck, 
        Shooter2022 shooter,
        Climber2022 climber,
        Dashboard2022 dashboard) {

        this(ledStrip, dashboard);
        
        this.llamaNeck = llamaNeck;
        this.shooter = shooter;
        this.climber = climber;
    }

    /**
     * For Testing, don't want to depend on other game pieces.
     * 
     * @param ledStrip the led strips
     */
    public Led2022UpdateCMD(Led2022 ledStrip, Dashboard2022 dashboard) {
        this.ledStrip = ledStrip;
        this.dashboard = dashboard;

        addRequirements(ledStrip);
        addRequirements(dashboard);

        rainbowTimer.start();
        purpleTimer.start();

        if (DriverStation.getAlliance() == Alliance.Red) {
            seeBallColor = COLORS_467.Red;
        } else {
            seeBallColor = COLORS_467.Blue;
        }
    }

    @Override
    public void initialize() {
        rainbowTimer.reset();
        purpleTimer.reset();
    }

    @Override
    public void execute() { 

        if (DriverStation.isEnabled()) {

            boolean seesTarget = HubTarget.hasTarget();
            double targetDistance = HubTarget.getDistance();
            double targetAngle = HubTarget.getAngle();
            double maxTargetAngle = Units.radiansToDegrees(Math.atan2(0.6, HubTarget.getDistance()));

            boolean seesBall = BallTracking.hasBall();
            double ballDistance = BallTracking.getDistance();
            double ballAngle = BallTracking.getAngle();

            if (seesTarget && targetDistance < TARGET_MAX_RANGE) {
                if (Math.abs(targetAngle) < maxTargetAngle) {
                    dashboard.setTargetStraight();
                } else if (targetAngle < 0.0) {
                    dashboard.setTargetLeft();
                } else {
                    dashboard.setTargetRight();
                }
            } else {
                dashboard.setTargetUnseen();
            }

            if (seesBall && ballDistance < BALL_MAX_RANGE) {
                if (ballAngle < BALL_MAX_ANGLE) {
                    dashboard.setSeeBallStraight();
                } else if (ballAngle < 0.0) {
                    dashboard.setSeeBallRight();
                } else {
                    dashboard.setSeeBallLeft();
                }
            } else {
                dashboard.setBallUnseen();
            }
            
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
            } else if (shooter != null && (shooter.getCurrentCommand() instanceof Shooter2022ShootCMD || shooter.getCurrentCommand() instanceof Shooter2022ShootSpeedCMD  || shooter.getCurrentCommand() instanceof Shooter2022ShootTargetCMD)) {
                if (HubTarget.hasTarget()) {
                    setColorMovingUp(Color.kBlack, Color.kGold);
                } else {
                    setColorMovingUp(Color.kAliceBlue, Color.kBlack);
                }
                
            } else if (llamaNeck != null && llamaNeck.hasLowerBall()) {
                dashboard.setHaveTwoBalls();
                if (seesTarget && targetDistance < TARGET_MAX_RANGE &&  Math.abs(targetAngle) < maxTargetAngle) {
                    setTop(seeTargetColor);
                    setBottom(seeTargetColor);
                } else {
                    set(hasBallColor);
                }
            } else if (llamaNeck != null && llamaNeck.hasUpperBall()) {
                dashboard.setHaveOneBall();
                setBottom(hasBallColor);
                if (seesTarget && targetDistance < TARGET_MAX_RANGE &&  Math.abs(targetAngle) < maxTargetAngle) {
                    setTop(seeTargetColor);
                } else if (seesBall && Math.abs(ballAngle) < BALL_MAX_ANGLE) {
                    setTop(seeBallColor);
                } else {
                    setTop(COLORS_467.Black); // Off
                }
            } else {
                dashboard.setHaveZeroBalls();
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

    public void setColorMovingUp(Color fgColor, Color bgColor) {
        if (purpleTimer.hasElapsed(SHOOTING_TIMER_SPEED * (RobotConstants.get().led2022LedCount() + 2))) {
            purpleTimer.reset();
        }

        for (int i = 0; i < RobotConstants.get().led2022LedCount(); i++) {
            if (purpleTimer.hasElapsed(SHOOTING_TIMER_SPEED * i)) {
                double timeUntilOff = Math.max(0, (SHOOTING_TIMER_SPEED * (i + 2)) - purpleTimer.get());
                double brightness = (255 * timeUntilOff);

                if (brightness == 0) {
                    ledStrip.setLED(i, bgColor);
                } else {
                    ledStrip.setRGB(i, (int) (fgColor.red * brightness), (int) (fgColor.green * brightness), (int) (fgColor.blue * brightness));
                }
             } else {
                ledStrip.setLED(i, bgColor);
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
