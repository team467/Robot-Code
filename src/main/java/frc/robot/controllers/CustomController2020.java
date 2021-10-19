package frc.robot.controllers;

public class CustomController2020 {
    public enum Buttons {
        CLIMB_LOCK(10),
        CLIMB_UP(11),
        CLIMB_DOWN(12);
    
        public final int value;
    
        Buttons(int value) {
            this.value = value;
        }
    }
}
