package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.*;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.*;
import java.util.stream.Stream;


public class Robot {
    private final DcMotor leftFront;
    private final DcMotor leftRear;
    private final DcMotor rightFront;
    private final DcMotor rightRear;
    private final DcMotor sbinPahar;

    private boolean turbo = true;

    private final BNO055IMU imu;

    private double headingOffset = 0.0;

    private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);

    public Robot(final HardwareMap hardwareMap) {
        leftFront = hardwareMap.dcMotor.get("lf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightFront = hardwareMap.dcMotor.get("rf");
        rightRear = hardwareMap.dcMotor.get("rr");

        sbinPahar = hardwareMap.dcMotor.get("sp");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

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

    public void move(double x, double y, double rotation) {
        final double direction = Math.atan2(x, y);
        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y));
        final double factorSin = speed * Math.sin(direction + Math.PI / 4.0);
        final double factorCos = speed * Math.cos(direction + Math.PI / 4.0);

        setMotors(factorSin + rotation, factorCos - rotation, factorCos + rotation, factorSin - rotation);
    }

    public void stop() {
        setMotors(0, 0, 0, 0);
    }

    // -----------------------
    // - Features functions
    // -----------------------
    public void sbin() {
        sbinPahar.setPower(0.5);
        scheduler.schedule(() -> sbinPahar.setPower(0), 3, TimeUnit.SECONDS);
    }

    public void cutTheRope() {
    }

    public void toggleTurbo() {
        turbo = !turbo;
    }

    public void TURBO() {
        turbo = true;
    }

    public boolean isTurbo() {
        return turbo;
    }

    private void setMotors(double lf, double lr, double rf, double rr) {
        final double divider = (turbo ? 1.0 : 7.5);
        final double scale = Stream.of(1.0, lf, lr, rf, rr).mapToDouble(Math::abs).max().getAsDouble();

        leftFront.setPower(lf / scale / divider);
        leftRear.setPower(lr / scale / divider);
        rightFront.setPower(rf / scale / divider);
        rightRear.setPower(rr / scale / divider);
    }

}