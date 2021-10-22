package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Data")
public class Data extends OpMode{
    private Robot robot;
    private Controller controller;

    @Override
    public void init(){
        robot = new Robot(hardwareMap);
        controller = new Controller(gamepad1);

    }

    public void loop(){
        controller.update();

        telemetry.clear();
        telemetry.addData("x: ", controller.leftStickX);
        telemetry.addData("y: ", controller.leftStickY);
        telemetry.update();

    }
}
