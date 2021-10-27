package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.sql.Time;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.util.stream.Stream;


public class Robot {
    private final DcMotor leftFront;
    private final DcMotor leftRear;
    private final DcMotor rightFront;
    private final DcMotor rightRear;
    private final DcMotor glass;
    private final DcMotor scissorsEngine;
    private final FlagController flagLeft, flagRight;

    private final Telemetry telemetry;

    private boolean turbo = false;

    private final BNO055IMU imu;

    private double headingOffset = 0.0;

    private final ScheduledExecutorService scheduler;
    private ScheduledFuture<?> lastGoCrazyAction = null;

    public boolean extendedArm = false;

    private Servo scissor;

    public Robot(final HardwareMap hardwareMap, final Telemetry t) {
        telemetry = t;
        leftFront = hardwareMap.dcMotor.get("lf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightFront = hardwareMap.dcMotor.get("rf");
        rightRear = hardwareMap.dcMotor.get("rr");
        scissorsEngine = hardwareMap.dcMotor.get("scissors");
        glass = hardwareMap.dcMotor.get("sp");
        flagLeft = new FlagController(hardwareMap.servo.get("left_flag"), 0.3, 0);
        flagRight = new FlagController(hardwareMap.servo.get("right_flag"), 0, 0.3);
        scissor = hardwareMap.servo.get("scissor");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);
        scissorsEngine.setDirection(DcMotorSimple.Direction.FORWARD);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        scheduler = Executors.newScheduledThreadPool(1);
    }

    // -------------------
    // - Encoding functions
    // -------------------
    public void runUsingEncoders() {
        setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void blockingRotate(double degree, double speed) {
        double initHeading = getAngularOrientation();

        rotate(speed);
        while(getAngularOrientation() - initHeading != Math.toRadians(degree));
        stop();
    }

    public void runWithoutEncoders() {
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private double getAngularOrientation() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }
    public void resetHeading() {
        headingOffset = getAngularOrientation();
    }

    // -------------------
    // - Motors functions
    // -------------------
    private void setMotorsMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftRear.setMode(mode);
        rightFront.setMode(mode);
        rightRear.setMode(mode);
    }

    public void rotate(double rotation) {
        setMotors(-rotation, -rotation, rotation, rotation);
    }

    public void move(double x, double y) {
        y = -y;

        final double orientation = getAngularOrientation();
        final double heading = (orientation - headingOffset) % (2.0 * Math.PI);
        headingOffset = orientation;

        final double direction = Math.atan2(x, y) - heading;
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));
        final double factorSin = speed * Math.sin(direction + Math.PI / 4.0);
        final double factorCos = speed * Math.cos(direction + Math.PI / 4.0);

        telemetry.addData("Movement", "X: %f\nY: %f\nOrientation: %f\n", x, y, orientation);

        setMotors(factorSin, factorCos, factorCos, factorSin);
    }

    public void stop() {
        setMotors(0, 0, 0, 0);
        if (lastGoCrazyAction != null && !lastGoCrazyAction.isDone()) {
            moveScissorsEngine(0);
            lastGoCrazyAction.cancel(true);
        }
    }

    // -----------------------
    // - Features functions
    // -----------------------
    public void shakeGlass() {
        glass.setPower(0.5);
    }

    public void stopShakingGlass() {
        glass.setPower(0.0);
    }

    public void initialCutPosition(){ scissor.setPosition(0.0); }
    public void cut() {
        initialCutPosition();
        scissor.setPosition(0.8);
        TimeUnit.SECONDS.sleep(1);
        initialCutPosition();
    }

    public void extendArm(){
        // 8 seconds
        if(extendedArm == false){
            scissorsEngine.setPower(0.5);
            TimeUnit.SECONDS.sleep(8);
            scissorsEngine.setPower(0);
            extendedArm = true;
        }
    }

    public void shrinkArm(){
        if(extendedArm == true){
            scissorsEngine.setPower(-0.5);
            TimeUnit.SECONDS.sleep(8);
            scissorsEngine.setPower(0);
            extendedArm = false;
        }
    }

    public void goCrazy() {
        if (lastGoCrazyAction != null && !lastGoCrazyAction.isDone()) {
            return;
        }

        // 1. spread stickers and other stuff
        shakeGlass();
        TimeUnit.SECONDS.sleep(2);
        stopShakingGlass();

        // 2. toggle the flags
        lastGoCrazyAction = scheduler.scheduleWithFixedDelay(() -> {
            flagLeft.toggle(0.7, 1);
            flagRight.toggle(0.7, 1);
        }, 0, 300, TimeUnit.MILLISECONDS);

        // 3. continous scissors moving
        for(int i = 0; i < 5; i++){
            cut();
        }

        // 4. the scissors arm moving up and down, side to side like a roller coaster


        // 5. rotate trying to drift
        rotate(0.7);
    }

    public void setTurbo(boolean value) {
        turbo = value;
    }

    public void moveScissorsEngine(double speed) {
        scissorsEngine.setPower(speed);
        telemetry.addData("Scissors move", "Position: %d, Target: %d, Speed: %f", scissorsEngine.getCurrentPosition(), scissorsEngine.getTargetPosition(), speed);
    }

    public boolean isTurbo() {
        return turbo;
    }

    private void setMotors(double lf, double lr, double rf, double rr) {
        final double scale = Stream.of(1.0, lf, lr, rf, rr).mapToDouble(Math::abs).max().getAsDouble();

        leftFront.setPower(getPower(lf, scale, "front left"));
        leftRear.setPower(getPower(lr, scale, "rear left"));
        rightFront.setPower(getPower(rf, scale, "front right"));
        rightRear.setPower(getPower(rr, scale, "rear right"));
    }

    private double getPower(double rf, double scale, String engine) {
        telemetry.addData(String.format("Power in %s", engine), "initial: %f; turbo: %b; scale: %f", rf, turbo, scale);
        return rf / (isTurbo() ? 1.0 : 2.0) / scale;
    }
}