package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;


public class Controller {
    private Gamepad gamepad;

    private int x, y, a, b;

    public double leftStickX, leftStickY;
    public double rightStickX, rightStickY;

    public Controller(Gamepad g){
        gamepad = g;
    }

    public void update(){
        leftStickX = gamepad.left_stick_x;
        leftStickY = gamepad.left_stick_y;
        rightStickX = gamepad.right_stick_x;
        rightStickY = gamepad.right_stick_y;
    }
    
}
