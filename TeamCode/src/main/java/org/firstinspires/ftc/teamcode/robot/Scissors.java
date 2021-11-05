package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Objects;
import java.util.concurrent.*;

public class Scissors {
    private final DcMotorEx arm;
    private final Servo scissors;
    private final ScheduledExecutorService scheduler;
    private final Telemetry telemetry;
    private final int armRaisedPosition;

    Scissors(@NonNull final Parameters params) {
        arm = Objects.requireNonNull(params.arm, "Scissors arm is not set");
        arm.setDirection(DcMotorSimple.Direction.REVERSE);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        scissors = Objects.requireNonNull(params.scissors, "Scissors servo is not set");
        scissors.setDirection(Servo.Direction.REVERSE);

        scheduler = Objects.requireNonNull(params.scheduler, "Scheduler is not set");
        telemetry = Objects.requireNonNull(params.telemetry, "Telemetry is not set");
        armRaisedPosition = params.armRaisedPosition;
    }

    private ScheduledFuture<?> lastMove = null, lastCut = null;

    /**
     * Raise or lower the scissors' arm. Note that the initial position of the arm
     * is considered the zero position - make sure the arm is positioned down before
     * starting the OpMode!
     *
     * @param positionPercentage Number in the interval [0.0, 1.0] that represents
     *                           how much the arm should be raised.
     * @return A future that completes when the arm is raised. Canceling this future
     * will stop the arm from moving.
     */
    public ScheduledFuture<?> moveArm(double positionPercentage) {
        if (!Utils.isDone(lastMove) && !lastMove.cancel(true)) {
            telemetry.addLine("Last move not done!");
            return null;
        }

        int targetPosition = (int) Math.floor(Utils.interpolate(0, armRaisedPosition, positionPercentage, 1));
        int initialPosition = arm.getCurrentPosition();
        if (targetPosition == initialPosition) {
            return null;
        }

        arm.setTargetPosition(targetPosition);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        arm.setPower(targetPosition > initialPosition ? 1 : -1);

        lastMove = Utils.poll(scheduler, () -> !arm.isBusy(), () -> arm.setPower(0), 10, TimeUnit.MILLISECONDS);

        return lastMove;
    }

    /**
     * Set the arm's motor power.
     *
     * Use this function to move the arm in its zero position, then restart the OpMode.
     *
     * @param power The desired power.
     */
    public void move(double power) {
        arm.setPower(power);
    }

    private static final double SCISSORS_POS = 0.3;

    /**
     * Do a cut.
     *
     * @return A future that completes when the scissors are back in their initial position.
     */
    public ScheduledFuture<?> cut() {
        if (!Utils.isDone(lastCut) && !lastCut.cancel(true)) {
            return null;
        }

        Servo s = scissors;
        s.setPosition(SCISSORS_POS);

        lastCut = scheduler.schedule(() -> s.setPosition(0), 500, TimeUnit.MILLISECONDS);

        return lastCut;
    }

    /**
     * Revert the scissors back to their initial position. Call this function if you want to
     * cancel a cut.
     */
    public void cancelCut() {
        if (Utils.isDone(lastCut) || !lastCut.cancel(true)) {
            return;
        }

        scissors.setPosition(0);
    }

    static class Parameters {
        DcMotorEx arm;
        Servo scissors;
        ScheduledExecutorService scheduler;
        Telemetry telemetry;
        int armRaisedPosition;
    }
}
