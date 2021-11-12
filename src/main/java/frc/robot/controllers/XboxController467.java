package frc.robot.controllers;

/**
 * Button and axes assignments for an XInput Controller.
 */
public final class XboxController467 {
    public enum Buttons {
        A(1),
        B(2),
        X(3),
        Y(4),
        BumperLeft(5),
        BumperRight(6),
        Back(7),
        Start(8),
        XBox(9),
        StickLeft(10),
        StickRight(11),
        POVup(12),
        POVright(13),
        POVdown(14),
        POVleft(15);
    
        public final int value;
    
        Buttons(int value) {
            this.value = value;
        }
    }

    public enum Axes {
        LeftX(0),
        LeftY(1),
        LeftTrigger(2),
        RightTrigger(3),
        RightX(4),
        RightY(5);

        public final int value;

        Axes(int value) {
            this.value = value;
        }
    }
}
