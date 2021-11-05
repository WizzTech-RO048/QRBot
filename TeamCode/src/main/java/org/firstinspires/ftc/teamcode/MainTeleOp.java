package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robot.*;

import java.util.concurrent.*;


@TeleOp
public class MainTeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new Robot(hardwareMap, telemetry);
        msStuckDetectStop = 5000;
    }

    private double angle = 0;
    private double lastAngleSet = 0, lastBowlSpeedSet = 0;
    private boolean isArmRaised = false;
    private double bowlSpeed = 0;
    private ScheduledFuture<?> lastRotation = null, lastScissorsArmRaise = null, lastCut = null;

    private double lastFlagsToggle = 0;

    @Override
    public void stop() {
        robot.wheels.stop();
        if (lastScissorsArmRaise != null) {
            lastScissorsArmRaise.cancel(true);
        }
        isArmRaised = false;

        if (lastCut != null) {
            lastCut.cancel(true);
        }

        robot.spinConfettiBowl(0);
        robot.flagFront.toggle(0, 0);
        robot.flagRear.toggle(0, 0);

        try {
            robot.scissors.moveArm(0).get();
        } catch (Exception ignored) {}
    }

    @Override
    public void loop() {
        double y = gamepad1.right_trigger - gamepad1.left_trigger;
        double x = Math.signum(gamepad1.right_stick_x) * Math.hypot(gamepad1.right_stick_x, gamepad1.right_stick_y);
        double r = gamepad1.left_stick_x;

        if (Utils.isDone(lastRotation)) {
            if (isZero(x) && isZero(y) && isZero(r)) {
                robot.wheels.stop();
            } else {
                robot.wheels.move(x, y, r);
            }
        }

        if (Utils.isDone(lastScissorsArmRaise) && gamepad1.y) {
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

            if (Utils.isDone(lastRotation) && gamepad1.x && angle != 0) {
                lastRotation = robot.wheels.rotateFor(angle);
                angle = 0;
            }
        }

        boolean allowToggleFlags = !Utils.inVicinity(lastFlagsToggle, time, 0.7);
        if (allowToggleFlags && gamepad1.b) {
            lastFlagsToggle = time;
            robot.flagFront.toggle();
            robot.flagRear.toggle();
        }

        double bowlSpeedIncr = 0;

        if (gamepad1.right_bumper) {
            bowlSpeedIncr = 0.1;
        } else if (gamepad1.left_bumper) {
            bowlSpeedIncr = -0.1;
        }

        if (bowlSpeedIncr != 0 && !Utils.inVicinity(lastBowlSpeedSet, time, 0.2)) {
            lastBowlSpeedSet = time;
            bowlSpeed = Utils.clamp(bowlSpeed + bowlSpeedIncr, 0, 1);
        }

        telemetry.addData("Bowl speed", bowlSpeed);

        robot.spinConfettiBowl(bowlSpeed);
    }

    private static boolean isZero(double value) {
        return Utils.inVicinity(value, 0, 0.01);
    }
}