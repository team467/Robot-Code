package frc.lib.autocheck;

import edu.wpi.first.wpilibj.Timer;

import java.util.Objects;

/**
 * The SubsystemFault class represents a fault or error that can occur in a subsystem.
 * It provides methods for creating, comparing, and hashing faults.
 */
public class SubsystemFault {
    public final String description;
    public final double timestamp;
    public final boolean isWarning;
    public final boolean sticky;

    /**
     * Creates a new SubsystemFault instance with the provided description and isWarning flag.
     * This constructor calls the other constructor in the same class with an additional argument set to false.
     *
     * @param description The description of the subsystem fault.
     * @param isWarning   A boolean flag indicating whether the fault is a warning or an error.
     */
    public SubsystemFault(String description, boolean isWarning) {
        this(description, isWarning, false);
    }

    /**
     * Creates a new SubsystemFault instance with the provided description.
     * This constructor calls the other constructor in the same class with the isWarning flag set to false.
     *
     * @param description The description of the subsystem fault.
     */
    public SubsystemFault(String description) {
        this(description, false);
    }

    /**
     * Creates a new SubsystemFault instance with the provided description, isWarning flag, and sticky flag.
     *
     * @param description The description of the subsystem fault.
     * @param isWarning   The flag indicating whether the fault is a warning.
     * @param sticky      The flag indicating whether the fault is sticky.
     */
    public SubsystemFault(String description, boolean isWarning, boolean sticky) {
        this.description = description;
        this.timestamp = Timer.getFPGATimestamp();
        this.isWarning = isWarning;
        this.sticky = sticky;
    }

    @Override
    public boolean equals(Object other) {
        if (this == other) {
            return true;
        }

        if (other == null) {
            return false;
        }

        if (getClass() != other.getClass()) {
            return false;
        }

        SubsystemFault otherSubsystemFault = (SubsystemFault) other;

        return description.equals(otherSubsystemFault.description)
                && isWarning == otherSubsystemFault.isWarning;
    }

    @Override
    public int hashCode() {
        return Objects.hash(description, timestamp, isWarning, sticky);
    }
}