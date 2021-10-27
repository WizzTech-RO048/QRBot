package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlagController {
    private final String flagName;
    private final Telemetry telemetry;
    private final Servo servo;
    private boolean raised = false;
    private final double startPos, endPos;

    FlagController(HardwareMap hm, Telemetry t, String name, double start, double end) {
        telemetry = t;
        flagName = name;
        servo = hm.servo.get(name);
        startPos = start;
        endPos = end;
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
        telemetry.addData(String.format("Flag %s status", flagName), "Raised: %b, Position: %f", raised, servo.getPosition());
    }

    private static double lerp(double a, double b, double p) {
        return a * (1 - p) + b * p;
    }
}
