package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.*;
import org.checkerframework.checker.nullness.qual.NonNull;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import java.util.concurrent.*;

import static java.util.concurrent.TimeUnit.*;

import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;

public class Wheels {
    private static final String[] HW_MOTOR_NAMES = {
            "wheelLeftFront",
            "wheelLeftRear",
            "wheelRightFront",
            "wheelRightRear"
    };

    private final double encoderTicksPerSecond;
    private final List<DcMotorEx> engines;
    private final double[] powerDistribution;

    private final BNO055IMU orientation;
    private final Telemetry telemetry;

    private final ScheduledExecutorService scheduler;

    private static DcMotorEx getEngine(HardwareMap map, String name) {
        DcMotorEx motor = map.get(DcMotorEx.class, name);
        motor.setMode(RunMode.STOP_AND_RESET_ENCODER);
        return motor;
    }

    Wheels(@NonNull final Parameters params) {
        this.telemetry = Objects.requireNonNull(params.telemetry, "Telemetry object was not set");
        this.orientation = Objects.requireNonNull(params.orientationSensor, "Orientation sensor was not set");
        this.scheduler = Objects.requireNonNull(params.scheduler, "Scheduler was not set");

        ArrayList<DcMotorEx> engines = new ArrayList<>();
        for (String name : HW_MOTOR_NAMES) {
            engines.add(getEngine(Objects.requireNonNull(params.hardwareMap, "Hardware map was not passed"), name));
        }

        // FIXME: This is a hack, it's a hardware problem that the left rear wheel spins in reverse.
        engines.get(1).setDirection(Direction.REVERSE);
        engines.get(2).setDirection(Direction.REVERSE);
        engines.get(3).setDirection(Direction.REVERSE);

        this.engines = engines;

        if (params.encoderResolution != 0 && params.rpm != 0) {
            this.encoderTicksPerSecond = (params.rpm / 60) * params.encoderResolution;
            useEncoders(true);
        } else {
            this.encoderTicksPerSecond = 0;
            useEncoders(false);
        }

        powerDistribution = new double[]{
                params.distributionLeftFront,
                params.distributionLeftRear,
                params.distributionRightFront,
                params.distributionRightRear
        };

        useBrakes(true);
    }

    private void forEachEngine(EngineFunction fn) {
        int index = 0;
        for (DcMotorEx engine : engines) {
            fn.apply(engine, index);
            index++;
        }
    }

    public void useEncoders(boolean shouldUse) {
        RunMode mode = shouldUse ? RunMode.RUN_USING_ENCODER : RunMode.RUN_WITHOUT_ENCODER;

        forEachEngine((engine, _i) -> engine.setMode(mode));
    }

    public void useBrakes(boolean shouldUse) {
        ZeroPowerBehavior behavior = shouldUse ? ZeroPowerBehavior.BRAKE : ZeroPowerBehavior.FLOAT;

        forEachEngine((engine, _i) -> engine.setZeroPowerBehavior(behavior));
    }

    /**
     * @param x The throttle on the horizontal axis of the robot.
     * @param y The throttle on the vertical axis of the robot.
     * @param r The rotation power around the robot's axis.
     */
    public void move(double x, double y, double r) {
        x = normalize(x);
        y = normalize(y);
        r = normalize(r);

        double[] input = {
                y + x + r, // left front
                y - x + r, // left rear
                y - x - r, // right front
                y + x - r  // right rear
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
        }

        forEachEngine((engine, i) -> setPower(engine, input[i] * powerDistribution[i]));
    }

    private void setPower(DcMotorEx engine, double power) {
        if (engine.getMode() == RunMode.RUN_USING_ENCODER) {
            engine.setVelocity(Math.round(power * encoderTicksPerSecond));
        } else {
            engine.setPower(power);
        }
    }

    private double getOrientation() {
         Orientation o = orientation.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
         return o.thirdAngle;
    }

    public ScheduledFuture<?> rotateFor(double degrees) {
        double startOrientation = getOrientation();
        double expectedOrientation = (startOrientation + degrees) % 360.0;

        telemetry.addData("Rotation", "Start orientation: %f, Expected orientation: %f", startOrientation, expectedOrientation);
        telemetry.update();

        // FIXME: Find why this does not do what it is intented to.

        move(0, 0, 0.5);

        return Utils.poll(
                scheduler,
                () -> Utils.isEqual(getOrientation(), expectedOrientation, 0.1),
                this::stop,
                10,
                MILLISECONDS
        );
    }

    public void stop() {
        forEachEngine((engine, _i) -> setPower(engine, 0.0));
    }

    private static double normalize(double val) {
        return Utils.clamp(val, -1, 1);
    }

    static class Parameters {
        public HardwareMap hardwareMap = null;
        public Telemetry telemetry = null;
        public BNO055IMU orientationSensor = null;
        public ScheduledExecutorService scheduler = null;

        /**
         * The PPR of the engine's encoder
         */
        public double encoderResolution = 0;
        /**
         * The rotations per minute of the engine
         */
        public double rpm = 0;

        /**
         * The percentage of the given power that is distributed to the engine on the front left side of the robot.
         */
        public double distributionLeftFront = 1;
        /**
         * The percentage of the given power that is distributed to the engine on the rear left side of the robot.
         */
        public double distributionLeftRear = 1;
        /**
         * The percentage of the given power that is distributed to the engine on the front right side of the robot.
         */
        public double distributionRightFront = 1;
        /**
         * The percentage of the given power that is distributed to the engine on the rear right side of the robot.
         */
        public double distributionRightRear = 1;
    }

    @FunctionalInterface
    private interface EngineFunction {
        void apply(DcMotorEx engine, int index);
    }
}
