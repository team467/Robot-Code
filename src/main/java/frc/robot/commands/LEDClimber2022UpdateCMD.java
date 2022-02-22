package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotConstants;
import frc.robot.subsystems.LEDClimber2022;

public class LEDClimber2022UpdateCMD extends CommandBase {
    private final double TIMER_SPEED = 0.006;

    private LEDClimber2022 ledClimber;
    private Color teamColor = Color.kBlue;
    private int color = 0;
    private Timer timer = new Timer();


    public LEDClimber2022UpdateCMD(LEDClimber2022 ledClimber) {
        this.ledClimber = ledClimber;
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

        if (lowerlimitswitchispressed==true) {
            setBottomPink();
        } else {
            setBottomOff();
        }

        if (upperlimitswitchispressed==true) {
            // setBottomPink();
            setTopPink();
        } else {
            setTopOff();
        }
 
        if (has zero balls, alliance is red, sees red ball) {
            setBottomRed();
            setTopRed();
        }

        if (has one ball, alliance is red, sees red ball) {
            setBottomPink();
            setTopRed();
        }

        if (has zero balls, alliance is blue, sees blue ball) {
            setBottomBlue();
            setTopBlue();
        }

        if (has one ball, alliance is blue, sees blue ball) {
            setBottomPink();
            setTopBlue();
        }

        if (has one ball, sees target) {
            setBottomPink();
            setTopGreen(); //need to make flash for only two seconds
        }

        if (has two balls, sees target) {
            setBottomPink();
            setTopPink();
            //make following code only flash for 2 seconds then go back to pink 
            setBottomGreen();
            setTopGreen();
        }

        if (has one ball, sees ball of alliance, sees target) {
            setBottomPink();
            //set top color of alliance AKA ignore target ();   
        }

        if (has two balls, sees ball of alliance) {
            setBottomPink();
            setTopPink();
            //ignore balls it sees, focuses on finding target 
        }

        if (shooting one or two balls) {
            setPurpleMovingUp();
        }

        if (climbing up) {
            setRainbowMovingUp();
            m_led.setData(m_ledBuffer);
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
    //??? need to fix
        if (timer.hasElapsed(TIMER_SPEED * (RobotConstants.get().ledClimber2022LEDCount() + 1))) {
            timer.reset();
        }

        for (int i = 0; i < RobotConstants.get().ledClimber2022LEDCount(); i++) {
            if (timer.hasElapsed(TIMER_SPEED * i)) {
                double timeUntilOff = Math.max(0, (TIMER_SPEED * (i + 1)) - timer.get());
                int brightness = (int) (255 * timeUntilOff);

                ledClimber.setRGB(i, 128 * brightness,0 * brightness,128 * brightness);
             }
        }
    }

    public void setRainbowMovingUp() {
        if (timer.hasElapsed(TIMER_SPEED * (RobotConstants.get().ledClimber2022LEDCount() + 1))) {
            timer.reset();
        }

        for (var i = 0; i < RobotConstants.get().ledClimber2022LEDCount(); i++) {
            if (timer.hasElapsed(TIMER_SPEED * i)) {
                double timeUntilOff = Math.max(0, (TIMER_SPEED * (i + 1)) - timer.get());
                int brightness = (int) (255 * timeUntilOff);

                ledClimber.setRGB(i, 128 * brightness,0 * brightness,128 * brightness);
             }
        }
    }
}
