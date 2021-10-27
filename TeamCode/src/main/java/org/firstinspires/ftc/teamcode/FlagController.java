package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlagController {
    private final String flagName;
    private final Telemetry telemetry;
    private final Servo servo;
    private boolean raised = false;

    FlagController(HardwareMap hm, Telemetry t, String name, Servo.Direction dir) {
        telemetry = t;
        flagName = name;
        servo = hm.servo.get(name);
        servo.setDirection(dir);
    }

    public void toggle() {
        raised = !raised;
        raise();
    }

    private void raise() {
        servo.setPosition(raised ? 0.3 : 0);
        telemetry.addData(String.format("Flag %s status", flagName), "Raised: %b, Position: %f", raised, servo.getPosition());
    }
}
