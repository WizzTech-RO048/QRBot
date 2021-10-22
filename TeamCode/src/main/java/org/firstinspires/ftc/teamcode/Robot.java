package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Robot {
    private DcMotor leftFront, leftRear, rightFront, rightRear;
    private DcMotor sbinPahar;

    private boolean _turbo;

    private final BNO055IMU imu;

    private double headingOffset = 0.0;
    private Orientation angles;
    private Acceleration gravity;

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


        _turbo = true;
    }

    // -------------------
    // - Encoding functions
    // -------------------
    public void runUsingEncoders(){ setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER, leftFront, leftRear, rightFront, rightRear); }
    public void runWithoutEncoders(){ setMotorMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER, leftFront, leftRear, rightFront, rightRear); }
    public boolean isGyroCalibrated() { return imu.isGyroCalibrated(); }

    // -------------------
    // - Heading functions
    // -------------------
    public double getRawHeading() { return angles.firstAngle; }
    public double getHeading() { return (getRawHeading() - headingOffset) % (2.0 * Math.PI); }
    public double getHeadingDegrees() { return Math.toDegrees(getHeading()); }
    public void resetHeading() { headingOffset = getRawHeading(); }

    public void loop(){
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        gravity = imu.getGravity();
    }

    // -------------------
    // - Motors functions
    // -------------------
    private void setMotorMode(DcMotor.RunMode mode, DcMotor ... motors){
        for(DcMotor motor : motors){
            motor.setMode(mode);
        }
    }


    public void movingRobot(double x, double y, double rotationValue){
        try{
            final double speed = Math.min(1.0, Math.sqrt(x * x + y * y)); // reading from the controller values
            final double rotation = Math.pow(rotationValue, 3.0);
            final double direction = Math.atan2(x, y) - getHeadingDegrees();

            final double lf = speed * Math.sin(direction + Math.PI / 4.0) - rotation;
            final double lr = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
            final double rf = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
            final double rr = speed * Math.sin(direction + Math.PI / 4.0) + rotation;

            setMotors(lf, lr, rf, rr, isTurbo());
            Thread.sleep(2000);
        } catch(Exception e) { }
    }

    public void moveForward() {
        setMotors(-1, -1, -1, -1, _turbo);
    }

    public void stop() {
        setMotors(0, 0, 0, 0, _turbo);
    }

    public void turn(double degrees) {
        setMotors(-1, -1, 1, 1, _turbo);

//        try {
//            Thread.sleep(1000);
//        } catch (Exception e) { e.printStackTrace(); }
//
//        stop();
    }

    public void moveTo(double speed, double direction) {

        final double lf = speed * Math.sin(direction + Math.PI / 4.0); // - rotation;
        final double rf = speed * Math.cos(direction + Math.PI / 4.0); // + rotation;
        final double lr = speed * Math.cos(direction + Math.PI / 4.0); // - rotation;
        final double rr = speed * Math.sin(direction + Math.PI / 4.0); // + rotation;

        setMotors(lf, lr, rf, rr, _turbo);
    }

    public void sbin(){
        double speed = 0.1;
        try {
            sbinPahar.setPower(0.5);
            Thread.sleep(3000);
            sbinPahar.setPower(0);
        } catch(Exception e){
            
        }
    }

    public void toggleTurbo() { _turbo = !_turbo; }
    public void TURBO() { _turbo = true; }
    public boolean isTurbo() { return _turbo; }

    private static double maxAbs(double ... xs){
        double ret = Double.MIN_VALUE;
        for(double x : xs){
            if(Math.abs(x) > ret){
                ret = Math.abs(x);
            }
        }
        return ret;
    }

    public void setMotors(double _lf, double _lr, double _rf, double _rr, boolean nitro) {
        final double divider = (nitro ? 1.0 : 7.5);
        final double scale = maxAbs(1.0, _lf, _lr, _rf, _rr);

        leftFront.setPower(_lf / scale / divider);
        leftRear.setPower(_lr / scale / divider);
        rightFront.setPower(_rf / scale / divider);
        rightRear.setPower(_rr / scale / divider);
    }

}
