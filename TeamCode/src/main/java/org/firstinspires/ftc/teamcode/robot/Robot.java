package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.*;


public class Robot {
    /**
     * The robot's wheels. Control its movement using the methods on this object.
     */
    public final Wheels wheels;
    /**
     * The robot's flags. Raise or lower them using these objects.
     */
    public final Flag flagFront, flagRear;
    /**
     * The robot's scissors mechanism. Raise or lower the scissors' arm and move the scissors themselves using this object.
     */
    public final Scissors scissors;

    private final DcMotor confettiBowl;
    private Servo cameraServo;


    private static final String
            HW_MOTOR_SCISSORS_ARM = "scissorsArm",
            HW_MOTOR_CONFETTI_BOWL = "confettiBowl",
            HW_SERVO_FLAG_FRONT = "flagFront",
            HW_SERVO_FLAG_REAR = "flagRear",
            HW_SERVO_SCISSORS = "scissors";

    private static final int SCISSORS_ARM_FINAL_POS = 12525;

    public Robot(HardwareMap map, Telemetry telemetry, ScheduledExecutorService scheduler) {
        BNO055IMU orientation = map.get(BNO055IMU.class, "imu");
        orientation.initialize(new BNO055IMU.Parameters());

        Wheels.Parameters wheelsParams = new Wheels.Parameters();
        wheelsParams.hardwareMap = map;
        wheelsParams.telemetry = telemetry;
        wheelsParams.orientationSensor = orientation;
        wheelsParams.scheduler = scheduler;
        wheelsParams.rpm = 435;
        wheelsParams.encoderResolution = 384.5;
        wheels = new Wheels(wheelsParams);

        Scissors.Parameters scissorsParams = new Scissors.Parameters();
        scissorsParams.arm = map.get(DcMotorEx.class, HW_MOTOR_SCISSORS_ARM);
        scissorsParams.scissors = map.servo.get(HW_SERVO_SCISSORS);
        scissorsParams.scheduler = scheduler;
        scissorsParams.telemetry = telemetry;
        scissorsParams.armRaisedPosition = SCISSORS_ARM_FINAL_POS;
        scissors = new Scissors(scissorsParams);

        confettiBowl = map.dcMotor.get(HW_MOTOR_CONFETTI_BOWL);
        confettiBowl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        Servo flagFrontServo = map.servo.get(HW_SERVO_FLAG_FRONT);
        flagFrontServo.setDirection(Servo.Direction.REVERSE);
        flagFront = new Flag(flagFrontServo, 0.7, 1);
        flagRear = new Flag(map.servo.get(HW_SERVO_FLAG_REAR), 0, 0.3);

        cameraServo = map.servo.get("cameraServo");
    }

    /**
     * Spin the confetti bowl.
     *
     * @param power The power to spin the conffeti bowl with. Provide zero so the bowl stops spinning.
     */
    public void spinConfettiBowl(double power) {
        confettiBowl.setPower(power);
    }

    public void cameraUp(){ cameraServo.setPosition(0.05); }
    public void cameraDown(){ cameraServo.setPosition(0.7); }
}