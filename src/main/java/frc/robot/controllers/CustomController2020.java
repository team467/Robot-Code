package frc.robot.controllers;

public class CustomController2020 {
    public enum Buttons {
        INTAKE_ARM(1),
        INTAKE_ROLLER_FORWARD(2),
        INTAKE_ROLLER_BACKWARD(3),
        INDEX_AUTO(4),
        INDEX_ROLLER_FORWARD(5),
        INDEX_ROLLER_BACKWARD(6),
        SHOOTER_AUTO(7),
        SHOOTER_FLYWHEEL(8),
        SHOOTER_SHOOT(9),
        CLIMBER_LOCK_SWITCH(10),
        CLIMBER_UP_BUTTON(11),
        CLIMBER_DOWN_BUTTON(12);
    
        public final int value;
    
        Buttons(int value) {
            this.value = value;
        }
    }
}
