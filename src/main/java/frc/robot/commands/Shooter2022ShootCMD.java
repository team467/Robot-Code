package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Spitter2022;
import frc.robot.subsystems.Trigger2022;

public class Shooter2022ShootCMD extends CommandBase {

    private final double TIME_UNTIL_FINISHED = 0.1;

    private final Trigger2022 trigger;
    private final LlamaNeck2022 llamaNeck;
    private final Spitter2022 spitter;

    private final Command llamaNeckStop;
    private final Command llamaNeckForward;

    private final Command triggerStop;
    private final Command triggerForward;
    private final Command triggerIdle;

    private final Command spitterForward;

    private final Timer timer;

    public Shooter2022ShootCMD(Trigger2022 trigger, LlamaNeck2022 llamaNeck, Spitter2022 spitter) {
        super();

        this.trigger = trigger;
        this.llamaNeck = llamaNeck;
        this.spitter = spitter;

        this.llamaNeckStop = new LlamaNeck2022StopCMD(llamaNeck);
        this.llamaNeckForward = new LlamaNeck2022ForwardCMD(llamaNeck);

        this.triggerStop = new Trigger2022StopCMD(trigger);
        this.triggerForward = new Trigger2022ForwardCMD(trigger);
        this.triggerIdle = new Trigger2022IdleCMD(trigger);

        this.spitterForward = new Spitter2022Forward(spitter);

        this.timer = new Timer();
    }

    @Override
    public void initialize() {
        triggerStop.schedule();
        llamaNeckStop.schedule();
        spitterForward.schedule();

        timer.start();
    }

    @Override
    public void execute() {
        if (spitter.atSpeed()) {
            triggerForward.schedule();
            llamaNeckForward.schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        if (llamaNeck.getUpperLimitSwitch() || llamaNeck.getLowerLimitSwitch()) {
            timer.reset();
        }
        return timer.hasElapsed(TIME_UNTIL_FINISHED);
    }
}
