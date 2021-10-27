package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.runUsingEncoders();
    }

    @Override
    public void stop() {
        robot.stop();
    }

    @Override
    public void loop() {
        robot.setTurbo(gamepad1.right_bumper);
        robot.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

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
    }
}
