package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends OpMode {
    private Robot robot;
    private Controller controller;

    @Override
    public void init(){
        robot = new Robot(hardwareMap);
        controller = new Controller(gamepad1);
    }

    private void updateRobot(){
        final double x = Math.pow(controller.leftStickX, 3.0);
        final double y = Math.pow(controller.leftStickY, 3.0);

        final double rotation = Math.pow(controller.rightStickX, 3.0);
        final double direction = Math.atan2(x, y) - robot.getHeading();
        final double speed = Math.min(1.0, Math.sqrt(x*x + y*y));

        final double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(controller.rightStickX), 1);
        final double lf = (y + x + controller.rightStickX) / denominator;
        final double lr = (y - x + controller.rightStickX) / denominator;
        final double rf = (y - x - controller.rightStickX) / denominator;
        final double rr = (y + x - controller.rightStickX) / denominator;

        // final double lf = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        // final double rf = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        // final double lr = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        // final double rr = speed * Math.sin(direction + Math.PI / 4.0) - rotation;

        // robot.setMotors(lf, lr, rf, rr, robot.isTurbo());
    }
    @Override
    public void loop(){
        controller.update();
        robot.headingLoop();

        double speedPercentage = 100;

        if(controller.A()){ robot.moveBackward(speedPercentage); }
        if(controller.X()){ robot.moveLeft(speedPercentage); }
        if(controller.Y()){ robot.moveForward(speedPercentage); }
        if(controller.B()){ robot.diagonalMovement(speedPercentage); }

        robot.stop();

        updateRobot();
    }
}
