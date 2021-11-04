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
    }

    private double angle = 0;
    private double lastAngleSet = 0;
    private boolean isArmRaised = false;
    private ScheduledFuture<?> lastRotation = null, lastScissorsArmRaise = null;

    private double lastFlagsToggle = 0;

    @Override
    public void stop() {
        robot.wheels.stop();
        if (lastScissorsArmRaise != null) {
            lastScissorsArmRaise.cancel(true);
        }
        isArmRaised = false;

        try {
            robot.scissors.moveArm(0).get();
        } catch (Exception ignored) {}
    }

    private boolean isRotating() {
        return lastRotation != null && !lastRotation.isDone();
    }

    @Override
    public void loop() {
        double y = gamepad1.right_trigger - gamepad1.left_trigger;
        double x = Math.signum(gamepad1.right_stick_x) * Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double r = gamepad1.left_stick_x;

        if (!isRotating()) {
            if (isZero(x) && isZero(y) && isZero(r)) {
                robot.wheels.stop();
            } else {
                robot.wheels.move(x, y, r);
            }
        }

        if ((lastScissorsArmRaise == null || lastScissorsArmRaise.isDone()) && gamepad1.y) {
            isArmRaised = !isArmRaised;
            telemetry.addData("Is arm raised", isArmRaised);
            lastScissorsArmRaise = robot.scissors.moveArm(isArmRaised ? 1 : 0);
        }

        boolean allowSetAngle = !Utils.inVicinity(lastAngleSet, time, 0.2);
        if (allowSetAngle) {
            if (gamepad1.dpad_right) {
                lastAngleSet = time;
                angle += 15;
            } else if (gamepad1.dpad_left) {
                lastAngleSet = time;
                angle -= 15;
            }

            telemetry.addData("Angle", angle);

            if (!isRotating() && gamepad1.x && angle != 0) {
                lastRotation = robot.wheels.rotateFor(angle);
                angle = 0;
            }
        }

        boolean allowToggleFlags = !Utils.inVicinity(lastFlagsToggle, time, 0.7);
        if (allowToggleFlags && gamepad1.b) {
            lastFlagsToggle = time;
            robot.flagLeft.toggle();
            robot.flagRight.toggle();
        }
    }

    private static boolean isZero(double value) {
        return Utils.inVicinity(value, 0, 0.01);
    }
}