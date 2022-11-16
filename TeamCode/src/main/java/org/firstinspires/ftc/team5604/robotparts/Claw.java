package org.firstinspires.ftc.team5604.robotparts;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Claw {
    private final double OPEN_POSITION;
    private final double CLOSE_POSITION;

    private boolean open;

    private Servo claw;

    public Claw(HardwareMap map, String name, double openPosition, double closePosition) {
        claw = map.get(Servo.class, name);

        OPEN_POSITION = openPosition;
        CLOSE_POSITION = closePosition;
    }

    public void toggle() {
        if(!open) {
            open();
        }
        else {
            close();
        }
    }

    public void close() {
        claw.setPosition(CLOSE_POSITION);
        open = false;
    }

    public void open() {
        claw.setPosition(OPEN_POSITION);
        open = true;
    }

    public boolean isOpen() {
        return open;
    }
}
