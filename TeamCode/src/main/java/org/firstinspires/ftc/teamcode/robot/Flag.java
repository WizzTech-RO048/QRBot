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

    /**
     * Fully raises or lowers the flag, depending on its current state.
     */
    public void toggle() {
        toggle(0, 1);
    }

    /**
     * Raise the flag at the end position or lower it at the start position,
     * depending on its current state.
     *
     * The positions are numbers in the range [0.0, 1.0] that indicate to which
     * percentage of the full range of motion to raise the flag to.
     *
     * @param start The position to lower the flag to, if the flag must be lowered.
     * @param end The position to raise the flag to, if the flag must be raised.
     */
    public void toggle(double start, double end) {
        raised = !raised;
        raise(start, end);
    }

    private void raise(double start, double end) {
        servo.setPosition(lerp(startPos, endPos, raised ? start : end));
    }

    private static double lerp(double a, double b, double p) {
        return Utils.interpolate(a, b, p, 1);
    }
}
