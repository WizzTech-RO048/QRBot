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
        if(gamepad.x){ x++; } else{ x = 0; }
        if(gamepad.y){ y++; } else{ y = 0; }
        if(gamepad.a){ a++; } else{ a = 0; }
        if(gamepad.b){ b++; } else{ b = 0; }

        leftStickX = gamepad.left_stick_x;
        leftStickY = gamepad.left_stick_y;
        rightStickX = gamepad.right_stick_x;
        rightStickY = gamepad.right_stick_y;
    }

    public boolean X() { return x > 0; }
    public boolean XOnce() { return x == 1; }

    public boolean Y() { return y > 0; }
    public boolean YOnce() { return y == 1; }

    public boolean A() { return a > 0; }
    public boolean AOnce() { return a == 0; }

    public boolean B() { return b > 0; }
    public boolean BOnce() { return b == 0; }
}
