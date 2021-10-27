package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends OpMode {
    private Robot robot;

    private Servo flag_left;
    private Servo flag_right;

    @Override
    public void init() {
        robot = new Robot(hardwareMap, telemetry);
        robot.runUsingEncoders();

        flag_left = hardwareMap.servo.get("left_flag");
        flag_right = hardwareMap.servo.get("right_flag");
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

        if(gamepad2.x) {
            flag_left.setPosition(0.2);
            flag_right.setPosition(0.3);
        }
        else {
            flag_left.setPosition(0.5);
            flag_right.setPosition(0);
        }
    }
}
