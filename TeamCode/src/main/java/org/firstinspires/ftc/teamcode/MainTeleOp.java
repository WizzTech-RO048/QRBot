package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init(){
        robot = new Robot(hardwareMap, telemetry);
    }

    @Override
    public void loop(){
        robot.setTurbo(gamepad1.right_trigger == 1);
        robot.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}
