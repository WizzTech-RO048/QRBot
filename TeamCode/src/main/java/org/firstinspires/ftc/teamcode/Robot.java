package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.stream.Stream;


public class Robot {
    private final DcMotor leftFront;
    private final DcMotor leftRear;
    private final DcMotor rightFront;
    private final DcMotor rightRear;
    private final DcMotor glass;
    private final DcMotor scissorsEngine;

    private final Telemetry telemetry;

    private boolean turbo = false;

    private final BNO055IMU imu;

    private double headingOffset = 0.0;

    public Robot(final HardwareMap hardwareMap, final Telemetry t) {
        telemetry = t;
        leftFront = hardwareMap.dcMotor.get("lf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightFront = hardwareMap.dcMotor.get("rf");
        rightRear = hardwareMap.dcMotor.get("rr");
        scissorsEngine = hardwareMap.dcMotor.get("scissors");
        glass = hardwareMap.dcMotor.get("sp");

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
    }

    // -------------------
    // - Encoding functions
    // -------------------
    public void runUsingEncoders() {
        setMotorsMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
        final double orientation = getAngularOrientation();
        final double heading = (orientation - headingOffset) % (2.0 * Math.PI);
        headingOffset = orientation;

        final double direction = heading - Math.atan2(x, y);
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));
        final double factorSin = speed * Math.sin(direction + Math.PI / 4.0);
        final double factorCos = speed * Math.cos(direction + Math.PI / 4.0);

        telemetry.addData("Movement", "X: %f\nY: %f\nOrientation: %f\n", x, y, orientation);

        setMotors(factorSin, factorCos, factorCos, factorSin);
    }

    public void stop() {
        setMotors(0, 0, 0, 0);
        telemetry.addData("Scissors position", scissorsEngine.getCurrentPosition());
        // TODO: Bring scissors arm down.
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

    public void cutTheRope() {
    }

    public void setTurbo(boolean value) {
        turbo = value;
    }

    public void moveScissorsEngine(double speed) {
        scissorsEngine.setPower(speed);
        telemetry.addData("Scissors move", "Position: %d, Speed: %f", scissorsEngine.getCurrentPosition(), speed);
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