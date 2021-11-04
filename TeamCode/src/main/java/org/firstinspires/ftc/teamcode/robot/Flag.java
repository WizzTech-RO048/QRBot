package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.*;

public class Flag {
    private final Servo servo;
    private boolean raised = false;
    private final double startPos, endPos;

    Flag(Servo s, double start, double end) {
        servo = s;
        startPos = start;
        endPos = end;

        s.setPosition(start);
    }

    public void toggle() {
        toggle(0, 1);
    }

    public void toggle(double start, double end) {
        raised = !raised;
        raise(start, end);
    }

    private void raise(double start, double end) {
        servo.setPosition(raised ? lerp(startPos, endPos, start) : lerp(startPos, endPos, end));
    }

    private static double lerp(double a, double b, double p) {
        return Utils.interpolate(a, b, p, 1);
    }
}
