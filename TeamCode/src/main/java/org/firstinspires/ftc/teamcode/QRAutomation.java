package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.os.Handler;

import androidx.annotation.NonNull;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureRequest;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSequenceId;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCaptureSession;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraException;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraFrame;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.Arrays;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.teamcode.robot.*;

@Autonomous(name = "QR Navigator")
public class QRAutomation extends LinearOpMode {
    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;
    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;

    private CameraCaptureSession cameraCaptureSession;
    private EvictingBlockingQueue<Bitmap> frameQueue;

    private Handler callbackHandler;

    private Robot robot;
    private final ScheduledExecutorService scheduler = Executors.newScheduledThreadPool(1);

    @Override
    public void runOpMode() throws InterruptedException {
        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        robot = new Robot(hardwareMap, telemetry, scheduler);

        initFrameQueue();

        if (!openCamera()) {
            telemetry.addData("ERROR", "cannot open camera").setRetained(true);
            telemetry.update();
            return;
        }

        if (!startCamera()) {
            telemetry.addData("ERROR", "cannot start camera").setRetained(true);
            telemetry.update();
            return;
        }

        telemetry.addData("Coniguration", "Done. Waiting for start...").setRetained(true);
        telemetry.update();

        waitForStart();

        telemetry.addData("OpMode", "Started").setRetained(true);
        telemetry.update();

        while (opModeIsActive()) {
            Bitmap b = frameQueue.poll();
            if (b == null) {
                continue;
            }

            if (gamepad1.b) {
                robot.wheels.stop();
            }

            onNewFrame(b);
        }

        robot.wheels.stop();
        robot.flagFront.toggle(0, 0);
        robot.flagRear.toggle(0, 0);
        robot.spinConfettiBowl(0);

        if (!Utils.isDone(lastArmMovement) && lastArmMovement.cancel(true)) {
            robot.scissors.moveArm(0).wait();
        }

        closeCamera();
    }

    private ScheduledFuture<?> lastArmMovement = null;

    // here is the instruction from the qr code
    private void onNewFrame(Bitmap frame) throws InterruptedException {
        String instruction = readQRCode(frame);

        if (instruction == null) {
            return;
        }

        telemetry.addData("Instruction", instruction).setRetained(true);
        telemetry.update();

        robot.wheels.stop();

        switch (instruction) {
            // in current circumstances, we are going to use case "2" as "forward" and case "1" as "backward"
            case "1":
                robot.wheels.move(0, 0.5, 0);
                break;
            case "2":
                robot.wheels.rotateFor(90).wait();
                break;
            case "3":
                robot.wheels.rotateFor(180).wait();
                break;
            case "4":
                robot.scissors.moveArm(1).wait();
                robot.scissors.cut().wait();
                lastArmMovement = robot.scissors.moveArm(0);
                robot.wheels.rotateFor(90).wait();
                break;
            case "5":
                if (!Utils.isDone(lastArmMovement) && !lastArmMovement.cancel(true)) {
                    return;
                }

                robot.scissors.moveArm(1).wait();
                robot.scissors.cut().wait();
                lastArmMovement = robot.scissors.moveArm(0);
                robot.wheels.rotateFor(-270).wait();
                break;
            case "6":
                robot.wheels.rotateFor(-135).wait();
                break;
            case "7":
                robot.spinConfettiBowl(0.5);
                robot.wheels.move(0, 0, 1);
                return;
        }

        robot.wheels.move(0, 0.6, 0);
    }


    // -------------------
    //    Camera logic
    // -------------------
    private void initFrameQueue() {
        frameQueue = new EvictingBlockingQueue<>(new ArrayBlockingQueue<>(2));
        frameQueue.setEvictAction(Bitmap::recycle);
    }

    private boolean openCamera() {
        if (camera != null) return false;
        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        return camera != null;
    }

    private boolean startCamera() throws InterruptedException {
        if (cameraCaptureSession != null) {
            return false;
        }

        final int imageFormat = ImageFormat.YUY2;
        CameraCharacteristics camCharacteristics = cameraName.getCameraCharacteristics();

        if (!contains(camCharacteristics.getAndroidFormats(), imageFormat)) {
            return false;
        }

        final Size size = camCharacteristics.getDefaultSize(imageFormat);
        final int fps = camCharacteristics.getMaxFramesPerSecond(imageFormat, size);
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override
                public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest, (session1, request, cameraFrame) -> {
                            Bitmap bmp = captureRequest.createEmptyBitmap();
                            cameraFrame.copyToBitmap(bmp);
                            frameQueue.offer(bmp);
                        }, Continuation.create(callbackHandler, (session12, cameraCaptureSequenceId, lastFrameNumber) -> RobotLog.ii("QR Test", "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber)));

                        synchronizer.finish(session);
                    } catch (CameraException | RuntimeException e) {
                        RobotLog.ee("QR Test", e, "exception starting capture");
                        telemetry.addData("ERROR: ", "exception starting capture");
                        telemetry.update();
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException | RuntimeException e) {
            synchronizer.finish(null);
            return false;
        }

        synchronizer.await();

        cameraCaptureSession = synchronizer.getValue();
        return true;
    }

    private void stopCamera() {
        if (cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }

    private void closeCamera() {
        stopCamera();
        if (camera != null) {
            camera.close();
            camera = null;
        }
    }

    // -------------------
    //       Utils
    // -------------------
    public String readQRCode(Bitmap bitmap) {
        int[] pixelsArray = new int[bitmap.getWidth() * bitmap.getHeight()];
        bitmap.getPixels(pixelsArray, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());
        LuminanceSource source = new RGBLuminanceSource(bitmap.getWidth(), bitmap.getHeight(), pixelsArray);
        BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));
        Reader qrCodesReader = new QRCodeReader();
        // centerX = bitmap.getWidth() / 2;
        // centerY = bitmap.getHeight() / 2;

        try {
            // printing the (x, y) coordinatas of the QR code
            // ResultPoint[] resultPoints = qrCodesReader.decode(binaryBitmap).getResultPoints();
            // int sizeOfResultPoint = resultPoints.length;
            // for (int i = 0; i < sizeOfResultPoint; i++) {
            //     String formatted = String.format("X: %f, Y: %f", resultPoints[i].getX(), resultPoints[i].getY());
            //     telemetry.addData(String.format("Point #%d", i), formatted);
            //     telemetry.update();
            // }

            // returning the string of the QR code
            return qrCodesReader.decode(binaryBitmap).getText();
        } catch (Exception e) {
            return null;
        }
    }

    private boolean contains(int[] array, int value) {
        for (int i : array) {
            if (i == value) {
                return true;
            }
        }
        return false;
    }
}
