package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
import edu.wpi.first.wpilibj.Relay.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotConstants;

public class HubCameraLED extends SubsystemBase {
    private Relay relay;

    public HubCameraLED() {
        super();

        relay = new Relay(RobotConstants.get().hubCameraLEDChannel(), Direction.kForward);
    }

    public void enable() {
        relay.set(Value.kForward);
    }

    public void disable() {
        relay.set(Value.kOff);
    }

    public boolean isEnabled() {
        return relay.get().equals(Value.kForward);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        super.initSendable(builder);

        builder.addBooleanProperty("LED Enabled", this::isEnabled, null);
    }
}

