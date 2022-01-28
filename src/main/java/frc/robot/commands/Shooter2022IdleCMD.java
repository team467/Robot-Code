package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LlamaNeck2022;
import frc.robot.subsystems.Spitter2022;
import frc.robot.subsystems.Trigger2022;

public class Shooter2022IdleCMD extends CommandBase {

    private final Trigger2022 trigger;
    private final LlamaNeck2022 llamaNeck;
    private final Spitter2022 spitter;

    private final Command llamaNeckStop;
    private final Command llamaNeckForward;

    private final Command triggerStop;
    private final Command triggerIdle;

    private final Command spitterStop;

    public Shooter2022IdleCMD(Trigger2022 trigger, LlamaNeck2022 llamaNeck, Spitter2022 spitter) {
        super();

        this.trigger = trigger;
        this.llamaNeck = llamaNeck;
        this.spitter = spitter;

        this.llamaNeckStop = new LlamaNeck2022StopCMD(llamaNeck);
        this.llamaNeckForward = new LlamaNeck2022ForwardCMD(llamaNeck);

        this.triggerStop = new Trigger2022StopCMD(trigger);
        this.triggerIdle = new Trigger2022IdleCMD(trigger);

        this.spitterStop = new Spitter2022StopCMD(spitter);
    }

    @Override
    public void initialize() {
        triggerIdle.schedule();
        llamaNeckForward.schedule();
        spitterStop.schedule();
    }

    @Override
    public void execute() {
        if (llamaNeck.getUpperLimitSwitch()) {
            triggerStop.schedule();

            if (llamaNeck.getLowerLimitSwitch()) {
                llamaNeckStop.schedule();
            }
        } else {
            triggerIdle.schedule();
            llamaNeckForward.schedule();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
