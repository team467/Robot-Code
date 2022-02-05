package frc.robot.controllers;

import java.util.ArrayList;

public class ControllerQueue {
    private static ControllerQueue instance = null;

    private ArrayList<CustomControllerBase> controllers = new ArrayList<>();

    public static ControllerQueue getInstance() {
        if (instance == null) instance = new ControllerQueue();

        return instance;
    }

    public void addController(CustomControllerBase controller) {
        controllers.add(controller);
    }

    public void run() {
        controllers.forEach(controller -> {
            controller.updateQueue();
        });
    }
}
