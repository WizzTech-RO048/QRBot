package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.HardwareMap;

// testing
public class Robot {
    private DcMotor leftFront, leftRear, rightFront, rightRear;

    private boolean _turbo;

    public Robot(final HardwareMap hardwareMap) {
        leftFront = hardwareMap.dcMotor.get("lf");
        leftRear = hardwareMap.dcMotor.get("lr");
        rightFront = hardwareMap.dcMotor.get("rf");
        rightRear = hardwareMap.dcMotor.get("rr");

        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        _turbo = true;
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

    public void toggleTurbo() { _turbo = !_turbo; }
    public void TURBO() { _turbo = true; }
    public boolean isTurbo() { return _turbo; }

    private void setMotors(double _lf, double _lr, double _rf, double _rr, boolean nitro) {
        final double precentage = nitro ? 1 : 0.35;

        if(_lf > 1.0 || _lr > 1.0 || _rf > 1.0 || _rr > 1.0) return ;
        if(_lf < -1.0 || _lr < -1.0 || _rf < -1.0 || _rr < -1.0) return ;

        leftFront.setPower(_lf * precentage);
        leftRear.setPower(_lr * precentage);
        rightFront.setPower(_rf * precentage);
        rightRear.setPower(_rr * precentage);
    }

}
