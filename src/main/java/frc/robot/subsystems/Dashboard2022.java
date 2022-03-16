package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Led2022UpdateCMD.COLORS_467;

public class Dashboard2022  extends SubsystemBase {

    private COLORS_467 hasBallColor = COLORS_467.White;
    private COLORS_467 seeTargetColor = COLORS_467.Gold;
    private COLORS_467 seeBallColor = COLORS_467.Blue;

    private static final int TARGET_INDICATOR_OFFSET_X = 7;
    private static final int TARGET_INDICATOR_OFFSET_Y = 0;
    private static final int SEE_BALL_INDICATOR_OFFSET_X = 7;
    private static final int SEE_BALL_INDICATOR_OFFSET_Y = 1;
    private static final int HAVE_BALL_INDICATOR_OFFSET_X = 8;
    private static final int HAVE_BALL_INDICATOR_OFFSET_Y = 2;

    private boolean targetFarLeft = false;
    private boolean targetNearLeft = false;
    private boolean targetNearRight = false;
    private boolean targetFarRight = false;

    private boolean seeBallFarLeft = false;
    private boolean seeBallNearLeft = false;
    private boolean seeBallNearRight = false;
    private boolean seeBallFarRight = false;

    private boolean hasBallOne = false;
    private boolean hasBallTwo = false;

    public void setTargetUnseen() {
        targetFarLeft = false;
        targetNearLeft = false;
        targetNearRight = false;
        targetFarRight = false;
    }

    public void setTargetRight() {
        targetFarLeft = false;
        targetNearLeft = false;
        targetNearRight = true;
        targetFarRight = true;
    }

    public void setTargetStraight() {
        targetFarLeft = false;
        targetNearLeft = true;
        targetNearRight = true;
        targetFarRight = false;
    }

    public void setTargetLeft() {
        targetFarLeft = true;
        targetNearLeft = true;
        targetNearRight = false;
        targetFarRight = false;
    }

    public void setBallUnseen() {
        seeBallFarLeft = false;
        seeBallNearLeft = false;
        seeBallNearRight = false;
        seeBallFarRight = false;
    }

    public void setSeeBallRight() {
        seeBallFarLeft = false;
        seeBallNearLeft = false;
        seeBallNearRight = true;
        seeBallFarRight = true;
    }

    public void setSeeBallStraight() {
        seeBallFarLeft = false;
        seeBallNearLeft = true;
        seeBallNearRight = true;
        seeBallFarRight = false;
    }

    public void setSeeBallLeft() {
        seeBallFarLeft = true;
        seeBallNearLeft = true;
        seeBallNearRight = false;
        seeBallFarRight = false;
    }

    public void setHaveZeroBalls() {
        hasBallOne = false;
        hasBallTwo = false;
    }

    public void setHaveOneBall() {
        hasBallOne = true;
        hasBallTwo = false;
    }

    public void setHaveTwoBalls() {
        hasBallOne = true;
        hasBallTwo = true;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        if (DriverStation.getAlliance() == Alliance.Red) {
            seeBallColor = COLORS_467.Red;
        } else {
            seeBallColor = COLORS_467.Blue;
        }

        ShuffleboardTab tab = Shuffleboard.getTab("Operator");
        Shuffleboard.selectTab("Operator");

        tab.addBoolean("Target Far Left", this::isTargetFarLeft)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(TARGET_INDICATOR_OFFSET_X + 0, TARGET_INDICATOR_OFFSET_Y)
            .withSize(1,1)
            .withProperties(Map.of(
                "Color when false", COLORS_467.Black.shuffleboard,
                "Color when true", seeTargetColor.shuffleboard));

        tab.addBoolean("Target Near Left", this::isTargetNearLeft)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(TARGET_INDICATOR_OFFSET_X + 1, TARGET_INDICATOR_OFFSET_Y)
            .withSize(1,1)
            .withProperties(Map.of(
                "Color when false", COLORS_467.Black.shuffleboard,
                "Color when true", seeTargetColor.shuffleboard));
        
        tab.addBoolean("Target Near Right", this::isTargetNearRight)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(TARGET_INDICATOR_OFFSET_X + 2, TARGET_INDICATOR_OFFSET_Y)
        .withSize(1,1)
        .withProperties(Map.of(
            "Color when false", COLORS_467.Black.shuffleboard,
            "Color when true", seeTargetColor.shuffleboard));

        tab.addBoolean("Target Far Right", this::isTargetFarRight)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(TARGET_INDICATOR_OFFSET_X + 3, TARGET_INDICATOR_OFFSET_Y)
            .withSize(1,1)
            .withProperties(Map.of(
                "Color when false", COLORS_467.Black.shuffleboard,
                "Color when true", seeTargetColor.shuffleboard));
    
        tab.addBoolean("See Ball Far Left", this::isSeeBallFarLeft)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(SEE_BALL_INDICATOR_OFFSET_X + 0, SEE_BALL_INDICATOR_OFFSET_Y)
            .withSize(1,1)
            .withProperties(Map.of(
                "Color when false", COLORS_467.Black.shuffleboard,
                "Color when true", seeBallColor.shuffleboard));

        tab.addBoolean("See Ball Near Left", this::isSeeBallNearLeft)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(SEE_BALL_INDICATOR_OFFSET_X + 1, SEE_BALL_INDICATOR_OFFSET_Y)
            .withSize(1,1)
            .withProperties(Map.of(
                "Color when false", COLORS_467.Black.shuffleboard,
                "Color when true", seeBallColor.shuffleboard));
        
        tab.addBoolean("See Ball Near Right", this::isSeeBallNearRight)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(SEE_BALL_INDICATOR_OFFSET_X + 2, SEE_BALL_INDICATOR_OFFSET_Y)
        .withSize(1,1)
        .withProperties(Map.of(
            "Color when false", COLORS_467.Black.shuffleboard,
            "Color when true", seeBallColor.shuffleboard));

        tab.addBoolean("See Ball Far Right", this::isSeeBallFarRight)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(SEE_BALL_INDICATOR_OFFSET_X + 3, SEE_BALL_INDICATOR_OFFSET_Y)
            .withSize(1,1)
            .withProperties(Map.of(
                "Color when false", COLORS_467.Black.shuffleboard,
                "Color when true", seeBallColor.shuffleboard));        

        tab.addBoolean("Has One Ball", this::isHasBallOne)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(HAVE_BALL_INDICATOR_OFFSET_X + 0, HAVE_BALL_INDICATOR_OFFSET_Y)
            .withSize(1,1)
            .withProperties(Map.of(
                "Color when false", COLORS_467.Black.shuffleboard,
                "Color when true", hasBallColor.shuffleboard));

        tab.addBoolean("Has Two Balls", this::isHasBallTwo)
            .withWidget(BuiltInWidgets.kBooleanBox)
            .withPosition(HAVE_BALL_INDICATOR_OFFSET_X + 3, HAVE_BALL_INDICATOR_OFFSET_Y)
            .withSize(1,1)
            .withProperties(Map.of(
                "Color when false", COLORS_467.Black.shuffleboard,
                "Color when true", hasBallColor.shuffleboard));        

    }

    public boolean isTargetFarLeft() {
        return targetFarLeft;
    }

    public boolean isTargetNearLeft() {
        return targetNearLeft;
    }

    public boolean isTargetNearRight() {
        return targetNearRight;
    }

    public boolean isTargetFarRight() {
        return targetFarRight;
    }

    public boolean isSeeBallFarLeft() {
        return seeBallFarLeft;
    }

    public boolean isSeeBallNearLeft() {
        return seeBallNearLeft;
    }

    public boolean isSeeBallNearRight() {
        return seeBallNearRight;
    }

    public boolean isSeeBallFarRight() {
        return seeBallFarRight;
    }

    public boolean isHasBallOne() {
        return hasBallOne;
    }

    public boolean isHasBallTwo() {
        return hasBallTwo;
    }
    
}
