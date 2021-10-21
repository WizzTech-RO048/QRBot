package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Data")
public class Data extends OpMode{
    private Controller controller;

    Telemetry telemetry;

    @Override
    public void init(){
        controller = new Controller(gamepad1);

    }

    public void loop(){
        controller.update();

        telemetry.addData("x: ", controller.leftStickX);
        telemetry.addData("y: ", controller.leftStickY);
        telemetry.update();

    }
}
