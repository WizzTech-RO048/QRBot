package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Data")
public class Data extends OpMode{
    private Controller controller;

    @Override
    public void init(){
        controller = new Controller(gamepad1);

    }
    
    public void loop(){
        controller.update();

    }
}
