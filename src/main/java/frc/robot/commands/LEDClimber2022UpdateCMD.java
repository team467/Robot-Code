package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.LEDClimber2022;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Spitter2022;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LEDClimber2022UpdateCMD extends CommandBase {
    private final double TIMER_SPEED = 0.35;

    private LEDClimber2022 ledClimber;
    private LlamaNeck2022 llamaNeck2022;
    private Spitter2022 spitter2022;
    private Color teamColor = Color.kBlue;
    private int color = 0;
    private Timer timer = new Timer();


    public LEDClimber2022UpdateCMD(Spitter2022 spitter2022, LlamaNeck2022 llamaNeck2022, LEDClimber2022 ledClimber) {
        this.ledClimber = ledClimber;
        this.spitter2022 = spitter2022;
        this.llamaNeck2022 = llamaNeck2022;
        timer.start();

        addRequirements(ledClimber);
    }

    @Override
    public void initialize() {

        if (DriverStation.getAlliance() == Alliance.Red) {
            teamColor = Color.kRed;
        } else {
            teamColor = Color.kBlue;
        }

        timer.reset();

    }

    @Override
    public void execute() { 
       
// note: look at chasing ball code to see how to tell robot our alliance; use for when seeing blue or red ball
// might need to make something that says if the lower limit switch isnt "held" then dont condsider it pressed (for when theres one ball which goes over the bottom switch)

        if (llamaNeck2022.hasUpperBall()) { 
            //has only one ball
            setBottomPink();
        } else {
            setBottomOff();
        }

        if (llamaNeck2022.hasLowerBall()) {
            //has two balls
            setBottomPink();
            setTopPink();
        } else {
            setTopOff();
        }
 
        if (DriverStation.getAlliance() == Alliance.Red && NetworkTableInstance.getDefault().getTable("Vision").getSubTable("BallTracking").getSubTable("Red").getEntry("HasBall").getBoolean(false)) {
           //has no balls but sees a red ball when the alliance is red
           //has no balls
            setBottomRed();
            setTopRed();
        }

        if (llamaNeck2022.hasUpperBall() && DriverStation.getAlliance() == Alliance.Red && NetworkTableInstance.getDefault().getTable("Vision").getSubTable("BallTracking").getSubTable("Red").getEntry("HasBall").getBoolean(false)) {
          //has one ball and alliance is red, sees red ball
            setBottomPink();
            setTopRed();
        }

        if (DriverStation.getAlliance() == Alliance.Blue && NetworkTableInstance.getDefault().getTable("Vision").getSubTable("BallTracking").getSubTable("Blue").getEntry("HasBall").getBoolean(false)) {
            //has zero balls and alliance is blue + sees a blue ball
            //has zero balls
            //NetworkTableInstance.getDefault().getTable("Vision").getSubTable("Red").getEntry("HasBall").getBoolean(false))
            setBottomBlue();
            setTopBlue();
        }

        if (llamaNeck2022.hasUpperBall() && DriverStation.getAlliance() == Alliance.Blue && NetworkTableInstance.getDefault().getTable("Vision").getSubTable("BallTracking").getSubTable("Blue").getEntry("HasBall").getBoolean(false)) {
            //has one ball, alliance is blue, and robot see's a blue ball
            setBottomPink();
            setTopBlue();
        }

        if (llamaNeck2022.hasUpperBall() && NetworkTableInstance.getDefault().getTable("Vision").getSubTable("HubTarget").getEntry("isValid").getBoolean(false)) {
            //robot has one ball and sees a target 
            setBottomPink();
            setTopGreen(); 
        }

        if (llamaNeck2022.hasLowerBall() && NetworkTableInstance.getDefault().getTable("Vision").getSubTable("HubTarget").getEntry("isValid").getBoolean(false)) {
            //robot already has two balls and sees target
            setBottomGreen();
            setTopGreen();
        }

        // e

        // if (llamaNeck2022.hasUpperBall()) {
        //     //idk
        //     //has one ball and sees ball of alliance, but sees target
        //     setBottomPink();
        //     //set top color of alliance AKA ignore target ();   
        // }

        // if (llamaNeck2022.hasLowerBall()) {
        //     //robot has two balls and sees ball of alliance

        //     //random note: ball tracking red valid 

        //     setBottomPink();
        //     setTopPink();
        //     //ignore balls it sees, focuses on finding target 
        // }

        // e

        if (spitter2022.getCurrentCommand() instanceof Spitter2022ForwardCMD) {
            //fix boolean?

            setPurpleMovingUp();
        }

        if (climber2022.getcurrentcmd) {
            //probably will have to import Climber2022 branch; its not in this branch for some reason???
            setRainbowMovingUp();
           // m_led.setData(m_ledBuffer);
        }

        ledClimber.sendData();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void setTopPink() {
        for (int i = RobotConstants.get().ledClimber2022LEDCount()/2 +1; i < RobotConstants.get().ledClimber2022LEDCount(); i++) {
            ledClimber.setRGB(i,255,192,203);
        }
    }

    public void setBottomPink() {
        for (int i = 0; i < RobotConstants.get().ledClimber2022LEDCount()/2; i++) {
            ledClimber.setRGB(i,255,192,203);
        }
    }

    public void setTopRed() {
        for (int i = RobotConstants.get().ledClimber2022LEDCount()/2 +1; i < RobotConstants.get().ledClimber2022LEDCount(); i++) {
            ledClimber.setRGB(i,255,0,0);
        }
    }

    public void setBottomRed() {
        for (int i = 0; i < RobotConstants.get().ledClimber2022LEDCount()/2; i++) {
            ledClimber.setRGB(i,255,0,0);
        }
    }

    public void setTopBlue() {
        for (int i = RobotConstants.get().ledClimber2022LEDCount()/2 +1; i < RobotConstants.get().ledClimber2022LEDCount(); i++) {
            ledClimber.setRGB(i,0,0,204);
        }
    }

    public void setBottomBlue() {
        for (int i = 0; i < RobotConstants.get().ledClimber2022LEDCount()/2; i++) {
            ledClimber.setRGB(i,0,0,204);
        }
    }

    public void setTopGreen() {
        for (int i = RobotConstants.get().ledClimber2022LEDCount()/2 +1; i < RobotConstants.get().ledClimber2022LEDCount(); i++) {
            ledClimber.setRGB(i,0,128,0);
        }
    }

    public void setBottomGreen() {
        for (int i = 0; i < RobotConstants.get().ledClimber2022LEDCount()/2; i++) {
            ledClimber.setRGB(i,0,128,0);
        }
    }

    public void setTopOff() {
        for (int i = RobotConstants.get().ledClimber2022LEDCount()/2 +1; i < RobotConstants.get().ledClimber2022LEDCount(); i++) {
            ledClimber.setRGB(i,0,0,0);
        }
    }

    public void setBottomOff() {
        for (int i = 0; i < RobotConstants.get().ledClimber2022LEDCount()/2; i++) {
            ledClimber.setRGB(i,0,0,0);
        }
    }

    public void setPurpleMovingUp() {
        if (timer.hasElapsed(TIMER_SPEED * (RobotConstants.get().ledClimber2022LEDCount() + 1))) {
            timer.reset();
        }

        for (int i = 0; i < RobotConstants.get().ledClimber2022LEDCount(); i++) {
            if (timer.hasElapsed(TIMER_SPEED * i)) {
                double timeUntilOff = Math.max(0, (TIMER_SPEED * (i + 1)) - timer.get());
                int brightness = (int) (255 * timeUntilOff);

                ledClimber.setRGB(i, 1 * brightness,0 * brightness, 1 * brightness);
             }
        }
    }

    public void setRainbowMovingUp() {
        if (timer.hasElapsed(TIMER_SPEED)) {
            color += 1;

            if (color > 360) color = 0;
            timer.reset();
        }
        
        for (int i = 0; i < RobotConstants.get().ledTower2022LEDCount(); i++) {
            ledClimber.setHSB(i, (color + (i * 360/RobotConstants.get().ledTower2022LEDCount())) % 360, 255, 127);
        }

        ledClimber.sendData();
    }
}
