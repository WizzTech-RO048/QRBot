package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class MainTeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.wheels.useEncoders(true);
    }

    @Override
    public void loop() {
        // TODO: Better control scheme

        double v = 0;
        if (gamepad1.dpad_up) {
            v = 1;
        } else if (gamepad1.dpad_down) {
            v = -1;
        }

        double h = 0;
        if (gamepad1.dpad_right) {
            h = 1;
        } else if (gamepad1.dpad_left) {
            h = -1;
        }

        double r = gamepad1.left_stick_x;

        if (isZero(v) && isZero(h) && isZero(r)) {
            robot.wheels.stop();
        } else {
            robot.wheels.move(v, h, r);
        }
    }

    private static boolean isZero(double value) {
        return Math.abs(value) < 0.01;
    }
}