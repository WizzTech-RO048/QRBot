package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.*;


public class Robot {
    public final Wheels wheels;
    public final Flag flagLeft, flagRight;

    private final DcMotor confettiBowl;
    private final DcMotor scissorsArm;
    private final Servo scissor;

    private final BNO055IMU orientation;

    private final Telemetry telemetry;

    private final ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();

    private static final String HW_MOTOR_SCISSORS_ARM = "scissorsArm";
    private static final String HW_MOTOR_CONFETTI_BOWL = "confettiBowl";
    private static final String HW_SERVO_FLAG_LEFT = "flagLeft";
    private static final String HW_SERVO_FLAG_RIGHT = "flagRight";
    private static final String HW_SERVO_SCISSORS = "scissors";

    public Robot(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;

        orientation = map.get(BNO055IMU.class, "imu");
        orientation.initialize(new BNO055IMU.Parameters());

        Wheels.Parameters params = new Wheels.Parameters();
        params.hardwareMap = map;
        params.telemetry = this.telemetry;
        params.orientationSensor = orientation;
        params.scheduler = scheduler;
        params.rpm = 435;
        params.encoderResolution = 384.5;
        wheels = new Wheels(params);

        confettiBowl = map.dcMotor.get(HW_MOTOR_CONFETTI_BOWL);

        scissorsArm = map.dcMotor.get(HW_MOTOR_SCISSORS_ARM);
        scissorsArm.setDirection(DcMotorSimple.Direction.REVERSE);

        scissor = map.servo.get(HW_SERVO_SCISSORS);

        // FIXME: Rename flags to flagFront and flagRear, fix one of the flag's positions.

        flagLeft = new Flag(map.servo.get(HW_SERVO_FLAG_LEFT), 0, 0.3);
        flagRight = new Flag(map.servo.get(HW_SERVO_FLAG_RIGHT), 0, 0.3);
    }

    public void moveScissorsArm(double power) {
        scissorsArm.setPower(power);
    }
}