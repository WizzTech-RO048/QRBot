package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.*;


public class Robot {
    public final Wheels wheels;
    public final Flag flagLeft, flagRight;
    public final Scissors scissors;
    private final DcMotor confettiBowl;

    private final BNO055IMU orientation;

    private final Telemetry telemetry;

    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();

    private static final String
            HW_MOTOR_SCISSORS_ARM = "scissorsArm",
            HW_MOTOR_CONFETTI_BOWL = "confettiBowl",
            HW_SERVO_FLAG_FRONT = "flagFront",
            HW_SERVO_FLAG_REAR = "flagRear",
            HW_SERVO_SCISSORS = "scissors";

    private static final int SCISSORS_ARM_FINAL_POS = 12525;

    public Robot(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;

        orientation = map.get(BNO055IMU.class, "imu");
        orientation.initialize(new BNO055IMU.Parameters());

        Wheels.Parameters wheelsParams = new Wheels.Parameters();
        wheelsParams.hardwareMap = map;
        wheelsParams.telemetry = this.telemetry;
        wheelsParams.orientationSensor = orientation;
        wheelsParams.scheduler = scheduler;
        wheelsParams.rpm = 435;
        wheelsParams.encoderResolution = 384.5;
        wheels = new Wheels(wheelsParams);

        Scissors.Parameters scissorsParams = new Scissors.Parameters();
        scissorsParams.arm = map.get(DcMotorEx.class, HW_MOTOR_SCISSORS_ARM);
        scissorsParams.scissors = map.servo.get(HW_SERVO_SCISSORS);
        scissorsParams.scheduler = scheduler;
        scissorsParams.telemetry = this.telemetry;
        scissorsParams.armRaisedPosition = SCISSORS_ARM_FINAL_POS;
        scissors = new Scissors(scissorsParams);

        confettiBowl = map.dcMotor.get(HW_MOTOR_CONFETTI_BOWL);

        // FIXME: fix one of the flag's positions.

        flagLeft = new Flag(map.servo.get(HW_SERVO_FLAG_FRONT), 0, 0.3);
        flagRight = new Flag(map.servo.get(HW_SERVO_FLAG_REAR), 0, 0.3);
    }

    public void spinConfettiBowl(double power) {
        confettiBowl.setPower(power);
    }
}