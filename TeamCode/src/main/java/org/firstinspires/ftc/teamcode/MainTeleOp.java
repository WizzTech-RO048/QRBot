package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends OpMode {
    private Robot robot;
    private FlagController flagLeft, flagRight;
    private double lastFlagsToggleTime = 0.0;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.runUsingEncoders();

        flagLeft = new FlagController(hardwareMap.servo.get("left_flag"), 0.3, 0);
        flagRight = new FlagController(hardwareMap.servo.get("right_flag"), 0, 0.3);
    }

    @Override
    public void stop() {
        robot.stop();
    }

    @Override
    public void loop() {
        robot.setTurbo(gamepad1.right_bumper);

        double x = gamepad1.right_stick_x;
        double y = gamepad1.right_trigger - gamepad1.left_trigger;

        if (y == 0) {
            robot.rotate(x);
        } else {
            robot.move(x, y);
        }

        if (gamepad1.y) {
            robot.shakeGlass();
        } else {
            robot.stopShakingGlass();
        }

        if (gamepad1.dpad_up) {
            robot.moveScissorsEngine(1);
        } else if (gamepad1.dpad_down) {
            robot.moveScissorsEngine(-1);
        } else {
            robot.moveScissorsEngine(0);
        }

        if (gamepad2.x) {
            if (lastFlagsToggleTime == 0.0 || (time - lastFlagsToggleTime) > 0.7) {
                flagLeft.toggle();
                flagRight.toggle();
                lastFlagsToggleTime = time;
            }
        }
    }
}
