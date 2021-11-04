package org.firstinspires.ftc.teamcode.robot;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.*;

public class Scissors {
    private final Parameters params;

    Scissors(@NonNull final Parameters params) {
        params.arm.setDirection(DcMotorSimple.Direction.REVERSE);
        params.arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        params.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.params = params;
    }

    private ScheduledFuture<?> lastMove = null;

    public ScheduledFuture<?> moveArm(double positionPercentage) {
        if (lastMove != null && !lastMove.isDone() && !lastMove.cancel(true)) {
            return null;
        }

        int targetPosition = (int) Math.floor(Utils.interpolate(0, params.armRaisedPosition, positionPercentage, 1));
        int initialPosition = params.arm.getCurrentPosition();
        if (targetPosition == initialPosition) {
            return null;
        }

        params.arm.setTargetPosition(targetPosition);
        params.arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        params.arm.setPower(targetPosition > initialPosition ? 1 : -1);

        lastMove = Utils.poll(params.scheduler, () -> !params.arm.isBusy(), () -> params.arm.setPower(0), 10, TimeUnit.MILLISECONDS);

        return lastMove;
    }

    static class Parameters {
        DcMotorEx arm;
        Servo scissors;
        ScheduledExecutorService scheduler;
        Telemetry telemetry;
        int armRaisedPosition;
    }
}
