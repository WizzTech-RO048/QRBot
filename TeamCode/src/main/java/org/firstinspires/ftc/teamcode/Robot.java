package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    public final DcMotor scissorsEngine;
    private final FlagController flagLeft, flagRight;

    private final Telemetry telemetry;

    private boolean turbo = false;

    private final BNO055IMU imu;
    private double headingOffset = 0.0;

    private final ScheduledExecutorService scheduler;
    private ScheduledFuture<?> lastGoCrazyAction = null;

    public boolean extendedArm = false;

    private Servo scissor;

    public Robot(final HardwareMap hardwareMap, final Telemetry telemetry) {
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
    public void runUsingEncoders() { setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER, leftFront, leftRear, rightFront, rightRear); }
    public void runWithoutEncoders() { setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, leftFront, leftRear, rightFront, rightRear); }

    public void blockingRotate(double degree, double speed) {
        double initHeading = getAngularOrientation();

        rotate(speed);

        while(Math.abs(Math.abs(Math.toDegrees(getAngularOrientation() - initHeading) % 360) - degree) > 5) {
            // telemetry.addData("Angular difference", Math.abs(Math.abs(Math.toDegrees(getAngularOrientation() - initHeading) % 360) - degree));
            // telemetry.update();
        }

        stop();
    }


    private double getRawHeading() { return angles.firstAngle; }
    public double getHeading() { return (getRawHeading() - headingOffset) % (2.0 * Math.PI); }
    public double getHeadingDegrees() { return Math.toDegrees(getHeading()); }
    public void resetHeading() { headingOffset = getRawHeading(); }
    private double getAngularOrientation() {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }


    // -------------------
    // - Motors functions
    // -------------------
    private void setMotorsMode(DcMotor.RunMode mode, DcMotor ... motors) {
        for(DcMotor motor : motors){
            motor.setMode(mode);
        }
    }

    public void rotate(double rotation) {
        setMotors(-rotation, -rotation, rotation, rotation);
    }

    public void move(double x1, double y1, double stickX) {

        final double x = Math.pow(x1, 3.0);
        final double y = Math.pow(y1, 3.0);

        final double rotation = Math.pow(stickX, 3.0);
        final double direction = Math.atan2(x, y) - getHeading();
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));

        final double orientation = getAngularOrientation();
        final double heading = (orientation - headingOffset) % (2.0 * Math.PI);
        headingOffset = orientation;

        final double lf = speed * Math.sin(direction + Math.PI / 4.0) + rotation;
        final double lr = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
        final double rf = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
        final double rr = speed * Math.sin(direction + Math.PI / 4.0) - rotation;
        telemetry.addData("Movement", "X: %f\nY: %f\nOrientation: %f\n", x, y, orientation);

        setMotors(lf, lr, rf, rr);
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

    public void initialCutPosition(){ scissor.setPosition(1); }

    public void cut() {
        initialCutPosition();
        sleep(1000);
        scissor.setPosition(0);
        sleep(1000);
        initialCutPosition();
    }

    public void extendArm(){
        // 8 seconds
        if(!extendedArm){
            scissorsEngine.setPower(0.5);
            // TimeUnit.SECONDS.sleep(8);
            sleep(16500);
            scissorsEngine.setPower(0);
            extendedArm = true;
        }
    }

    public void shrinkArm(){
        if(extendedArm){
            scissorsEngine.setPower(-0.5);
            sleep(16500);
            scissorsEngine.setPower(0);
            extendedArm = false;
        }
    }

    public void goCrazy() {
        if (lastGoCrazyAction != null && !lastGoCrazyAction.isDone()) {
            return;
        }

        // spread stickers and other stuff
        shakeGlass();

        // toggle the flags
        lastGoCrazyAction = scheduler.scheduleWithFixedDelay(() -> {
            flagLeft.toggle(0.7, 1);
            flagRight.toggle(0.7, 1);
        }, 0, 300, TimeUnit.MILLISECONDS);

        // continous scissors moving
        for(int i = 0; i < 5; i++){
            cut();
        }

        // rotating trying to drift
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

    private void setMotorsVelocity(double lf, double lr, double rf, double rr) {
        // final double scale = Stream.of(1.0, lf, lr, rf, rr).mapToDouble(Math::abs).max().getAsDouble();

        leftFront.setVelocity(lf);
        leftRear.setVelocity(lr);
        rightFront.setVelocity(rf);
        rightRear.setVelocity(rr);
    }

    private static double maxAbs(double ... xs){
        double ret = Double.MIN_VALUE;
        for(double x : xs){
            if(Math.abs(x) > ret){
                ret = Math.abs(x);
            }
        }
        return ret;
    }

    private void setMotors(double _lf, double _lr, double _rf, double _rr) {
//        final double scale = Stream.of(1.0, lf, lr, rf, rr).mapToDouble(Math::abs).max().getAsDouble();
//
//        leftFront.setPower(getPower(lf, scale, "front left"));
//        leftRear.setPower(getPower(lr, scale, "rear left"));
//        rightFront.setPower(getPower(rf, scale, "front right"));
//        rightRear.setPower(getPower(rr, scale, "rear right"));

        final double divider = (isTurbo() ? 1.0 : 7.5);
        final double scale = maxAbs(1.0, _lf, _lr, _rf, _rr);

        leftFront.setPower(_lf / scale / divider);
        leftRear.setPower(_lr / scale / divider);
        rightFront.setPower(_rf / scale / divider);
        rightRear.setPower(_rr / scale / divider);
    }

    private double getPower(double rf, double scale, String engine) {
        telemetry.addData(String.format("Power in %s", engine), "initial: %f; turbo: %b; scale: %f", rf, turbo, scale);
        return rf / (isTurbo() ? 1.0 : 2.0) / scale;
    }

     public void sleep(int milis){
        try{
            Thread.sleep(milis);
        } catch(Exception e) { }
     }
}