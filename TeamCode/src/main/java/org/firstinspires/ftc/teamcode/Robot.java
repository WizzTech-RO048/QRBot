package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;


public class Robot {
    static class Wheels {
        private static final String
                HW_MOTOR_LEFT_FRONT = "wheelLeftFront",
                HW_MOTOR_LEFT_REAR = "wheelLeftRear",
                HW_MOTOR_RIGHT_FRONT = "wheelRightFront",
                HW_MOTOR_RIGHT_REAR = "wheelRightRear";

        private static DcMotorEx getMotor(HardwareMap map, String name) {
            DcMotorEx motor = map.get(DcMotorEx.class, name);
            // Reverse direction because it seems that the wheels spin opposite
            // to what is set by the program.
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            return motor;
        }

        private final DcMotorEx leftFront, leftRear, rightFront, rightRear;
        private final Telemetry telemetry;

        private final double encoderTicksPerSecond;

        Wheels(HardwareMap map, Telemetry telemetry, double encoderResolution, double rpm) {
            leftFront = getMotor(map, HW_MOTOR_LEFT_FRONT);
            leftRear = getMotor(map, HW_MOTOR_LEFT_REAR);
            rightFront = getMotor(map, HW_MOTOR_RIGHT_FRONT);
            rightRear = getMotor(map, HW_MOTOR_RIGHT_REAR);

            this.telemetry = telemetry;
            this.encoderTicksPerSecond = (rpm / 60) * encoderResolution;

            useEncoders(true);
        }

        public void useEncoders(boolean shouldUse) {
            DcMotor.RunMode mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
            if (shouldUse) {
                mode = DcMotor.RunMode.RUN_USING_ENCODER;
            }

            leftFront.setMode(mode);
            leftRear.setMode(mode);
            rightFront.setMode(mode);
            rightRear.setMode(mode);
        }

        /**
         * @param v The throttle on the vertical axis of the robot.
         * @param h The throttle on the horizontal axis of the robot.
         * @param r The rotation power around the robot's axis.
         */
        public void move(double v, double h, double r) {
            v = normalize(v);
            h = normalize(h);
            r = -normalize(r); // This has to be reversed due to the wheels spinning in reverse.

            telemetry.addData("Input", "X: %f, Y: %f, R: %f", v, h, r);

            // TODO: Formulas? Robot does not seem to rotate correctly when vertical throttle is applied.

            double[] input = {
                    v + h - r, // left front
                    v - h - r, // left rear
                    v - h + r, // right front
                    v + h + r  // right rear
            };

            double highest = 0;

            for (double d : input) {
                double abs = Math.abs(d);
                if (abs > highest) {
                    highest = abs;
                }
            }

            highest = Math.max(highest, 1);

            for (int i = 0; i < input.length; i++) {
                input[i] /= highest;
                telemetry.addData(String.format("Engine %d", i), input[i]);
            }

            setPower(leftFront, input[0]);
            setPower(leftRear, input[1]);
            setPower(rightFront, input[2]);
            setPower(rightRear, input[3]);
        }

        private void setPower(DcMotorEx engine, double power) {
            engine.setVelocity(power * encoderTicksPerSecond);
        }

        public void stop() {
            setPower(leftFront, 0.0);
            setPower(leftRear, 0.0);
            setPower(rightFront, 0.0);
            setPower(rightRear, 0.0);
        }

        private static double normalize(double val) {
            return Math.max(-1, Math.min(val, 1));
        }
    }

    public final Wheels wheels;
    private final DcMotor confettiBowl;
    private final DcMotor scissorsArm;
    private final Servo scissor;
    private final FlagController flagLeft, flagRight;

    private final BNO055IMU orientation;

    private final Telemetry telemetry;

    private static final String HW_MOTOR_SCISSORS_ARM = "scissorsArm";
    private static final String HW_MOTOR_CONFETTI_BOWL = "confettiBowl";
    private static final String HW_SERVO_FLAG_LEFT = "flagLeft";
    private static final String HW_SERVO_FLAG_RIGHT = "flagRight";
    private static final String HW_SERVO_SCISSORS = "scissors";

    public Robot(final HardwareMap hardwareMap, final Telemetry t) {
        wheels = new Wheels(hardwareMap, t, 384.5, 435);
        confettiBowl = hardwareMap.dcMotor.get(HW_MOTOR_CONFETTI_BOWL);
        scissorsArm = hardwareMap.dcMotor.get(HW_MOTOR_SCISSORS_ARM);
        scissor = hardwareMap.servo.get(HW_SERVO_SCISSORS);
        flagLeft = new FlagController(hardwareMap.servo.get(HW_SERVO_FLAG_LEFT), 0.3, 0);
        flagRight = new FlagController(hardwareMap.servo.get(HW_SERVO_FLAG_RIGHT), 0, 0.3);

        orientation = hardwareMap.get(BNO055IMU.class, "imu");
        orientation.initialize(new BNO055IMU.Parameters());

        telemetry = t;
    }
}