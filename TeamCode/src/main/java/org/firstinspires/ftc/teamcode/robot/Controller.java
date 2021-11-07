package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.Gamepad;

public class Controller{
    private final Gamepad gamepad;

    private int x, y, a, b;
    private double leftStickX, leftStickY;
    private double rightStickX, rightStickY;

    public Controller(Gamepad g){
        gamepad = g;
    }

    public void update(){
        if(gamepad.x){ x++; } else{ x = 0; }
        if(gamepad.y){ y++; } else{ y = 0; }
        if(gamepad.a){ a++; } else{ a = 0; }
        if(gamepad.b){ b++; } else{ b = 0; }

        leftStickX = gamepad.left_stick_x;
        leftStickY = gamepad.left_stick_y;
        rightStickX = gamepad.right_stick_x;
        rightStickY = gamepad.right_stick_y;
    }

    public boolean x(){ return x > 0; }
    public boolean xonce(){ return x == 1; }

    public boolean y(){ return y > 0; }
    public boolean yonce(){ return y == 1; }

    public boolean a(){ return a > 0; }
    public boolean aonce(){ return a == 1; }

    public boolean b(){ return b > 0; }
    public boolean bonce(){ return b == 1; }

}
