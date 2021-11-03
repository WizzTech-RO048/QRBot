package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.*;

import java.util.concurrent.ScheduledFuture;


@TeleOp
public class MainTeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.wheels.useEncoders(true);
    }

    private double angle = 0;
    private double lastAngleSet = 0;
    private ScheduledFuture<?> lastRotation = null;

    private double lastFlagsToggle = 0;

    @Override
    public void stop() {
        robot.wheels.stop();
        robot.flagRight.toggle(0, 0);
        robot.flagLeft.toggle(0, 0);
    }

    @Override
    public void loop() {
        double y = gamepad1.right_trigger - gamepad1.left_trigger;
        double x = gamepad1.right_stick_x * 1.1;
        double r = gamepad1.left_stick_x;

        if (isZero(x) && isZero(y) && isZero(r)) {
            robot.wheels.stop();
        } else {
            robot.wheels.move(x, y, r);
        }

        if (gamepad1.dpad_up) {
            robot.moveScissorsArm(-1);
        } else if (gamepad1.dpad_down) {
            robot.moveScissorsArm(1);
        } else {
            robot.moveScissorsArm(0);
        }

        boolean allowSetAngle = !Utils.isEqual(lastAngleSet, time, 0.2);
        if (allowSetAngle) {
            if (gamepad1.dpad_right) {
                lastAngleSet = time;
                angle += 5;
            } else if (gamepad1.dpad_left) {
                lastAngleSet = time;
                angle -= 5;
            }

            if ((lastRotation == null || lastRotation.isDone()) && gamepad1.x && angle != 0) {
                lastRotation = robot.wheels.rotateFor(angle);
                angle = 0;
            }
        }

        boolean allowToggleFlags = !Utils.isEqual(lastFlagsToggle, time, 0.7);
        if (allowToggleFlags && gamepad1.b) {
            lastFlagsToggle = time;
            robot.flagLeft.toggle();
            robot.flagRight.toggle();
        }
    }

    private static boolean isZero(double value) {
        return Utils.isEqual(value, 0, 0.01);
    }
}