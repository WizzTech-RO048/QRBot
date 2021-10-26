package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends OpMode {
    private Robot robot;

    @Override
    public void init(){
        robot = new Robot(hardwareMap);
    }

    @Override
    public void loop(){
        robot.move(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
    }
}
