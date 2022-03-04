package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    private COLORS_467 idleColorTop = COLORS_467.Blue;
    private COLORS_467 idleColorBottom = COLORS_467.Gold;
    private COLORS_467 hasBallColor = COLORS_467.White;
    private COLORS_467 seeTargetColor = COLORS_467.Gold;
    private COLORS_467 seeBallColor = COLORS_467.Blue;

    public enum COLORS_467 {
        White(0xFF, 0xFF, 0xFF),
        Red(0xFF, 0x00, 0x00),
        Green(0x00, 0x80, 0x00),
        Blue(0x00, 0x00, 0xCC),
        Gold(0xFF, 0xC2, 0x0A),
        Pink(0xFF,192,203),
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
        this.ledStrip = ledStrip;
        this.spitter = spitter;
        this.llamaNeck = llamaNeck;

        timer.start();

        addRequirements(ledStrip);
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

    }

    @Override
    public void initialize() {

        if (DriverStation.getAlliance() == Alliance.Red) {
            seeBallColor = COLORS_467.Red;
        } else {
            seeBallColor = COLORS_467.Blue;
        }

        // ShuffleboardTab tab = Shuffleboard.getTab("Operator");
        // SuppliedValueWidget<Boolean> rightUpper = tab.addBoolean("Right Status",() -> { return true; })
        //     .withWidget(BuiltInWidgets.kBooleanBox);
        // rightUpper.withProperties(Map.of("colorWhenTrue", Color.kGold));


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

        if (climber != null && climber.isEnabled()) {
            setRainbowMovingUp();
//        } else if (spitter != null && spitter.isAtShootingSpeed()) {
//            //spitter.getCurrentCommand() instanceof Spitter2022ForwardCMD
//            setPurpleMovingUp();
        } else if (llamaNeck != null && llamaNeck.hasLowerBall()) {
            if (seesTarget && targetDistance < 3.0 &&  Math.abs(targetAngle) < 4.0) {
                setTop(seeTargetColor);
                setBottom(seeTargetColor);
            } else {
                set(hasBallColor);
            }
        } else if (llamaNeck != null && llamaNeck.hasUpperBall()) {
            setBottom(hasBallColor);
            if (seesBall && ballDistance < 3.0 && Math.abs(ballAngle) < 10.0) {
                set(seeBallColor);
            } else if (seesTarget && targetDistance < 3.0 &&  Math.abs(targetAngle) < 4.0) {
                setTop(seeTargetColor);
            } else {
                setTop(COLORS_467.Black); // Off
            }
        } else {
            if (seesBall && ballDistance < 3.0 && Math.abs(ballAngle) < 10.0) {
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
