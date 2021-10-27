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
import com.google.zxing.Result;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.android.util.Size;
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
import java.util.Objects;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

@Autonomous(name = "QR Navigator")
public class QR_Nav extends LinearOpMode {

    String filePath = "F:\\Opulent_ProjectsDirectory_2015-2016\\.metadata\\.plugins\\org.eclipse.wst.server.core\\tmp0\\wtpwebapps\\AttendanceUsingQRCode\\QRCodes\\student3232_2015_12_15_10_29_46_123";
    String charset = "UTF-8"; // or "ISO-8859-1"

    private static final int secondsPermissionTimeout = Integer.MAX_VALUE;
    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;

    private CameraCaptureSession cameraCaptureSession;
    private EvictingBlockingQueue<Bitmap> frameQueue;

    private Handler callbackHandler;
    private FuckingViewPort webcamViewport;

    private Robot robot;
    private Servo gripper;

    private String lastInstruction = null;
    private final Reader qrCodesReader = new QRCodeReader();

    @Override
    public void runOpMode() throws InterruptedException {

        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        robot = new Robot(hardwareMap, telemetry);

        gripper = hardwareMap.servo.get("gripper");
        gripper.setPosition(0.0);

        // cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        initFrameQueue(2);

        try {
            openCamera();
            if (camera == null) {
                telemetry.addData("ERROR: ", "cannot open camera");
                telemetry.update();
                return;
            }
            startCamera();
            if (cameraCaptureSession == null) {
                telemetry.addData("ERROR: ", "cannot start camera");
                telemetry.update();
                return;
            }
            waitForStart();

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcamViewport = new FuckingViewPort(cameraMonitorViewId);

            telemetry.clear();
            telemetry.addData("> ", "Started");
            telemetry.update();

            while (opModeIsActive()) {
                if (gamepad1.a) gripper.setPosition(0.7);
                if (gamepad1.b) gripper.setPosition(0.3);

                Bitmap bmp = frameQueue.poll();

                if (bmp != null) {
                    // webcamViewport.draw(bmp);
                    onNewFrame(bmp);
                }
                telemetry.update();
            }
        } finally {
            closeCamera();
        }
    }

    // here is the instruction from the qr code
    private void onNewFrame(Bitmap frame) {
        String instruction = readQRCode(frame);

        if (instruction == null) {
            return;
        }

        telemetry.addData("Instruction", instruction);
        telemetry.addData("Turbo", robot.isTurbo());
        // telemetry.addData("heading:", robot.getHeading());
        telemetry.update();

        double x = 0;
        double y = 0;
        double degree = 0;

        switch (instruction) {
            // in current circumstances, we are going to use case "2" as "forward" and case "1" as "backward"
            case "1":
                x = 0.0;
                y = 0.5;
                degree = 0.0;
                break;   // move backward
            case "2":
                x = 0.0;
                y = -0.5;
                degree = 0.0;
                break;  // move forward
            case "3":
                x = 0.5;
                y = 0.0;
                degree = 0.0;
                break;   // move right
            case "4":
                x = -0.5;
                y = 0.0;
                degree = 0.0;
                break;  // move left
            case "5":
                x = 0.0;
                y = 0.0;
                degree = 45.0;
                break;  // rotate right
            case "6":
                x = 0.0;
                y = 0.0;
                degree = -1.0;
                break;  // rotate left
            case "7":
                robot.stop();
                robot.cutTheRope();
                break; // cut the rope
            case "8":
                robot.resetHeading();
                break;
        }

        if (degree != 0) {
            robot.rotate(Math.toRadians(degree));
        } else {
            robot.move(x, y);
        }

        try {
            if (instruction.equals(lastInstruction)) {
                robot.stop();
                Thread.sleep(1000); // ???
            }
        } catch (Exception ignored) {

        } finally {
            lastInstruction = instruction;
        }
    }

    // -------------------
    //    Camera logic
    // -------------------
    private void initFrameQueue(int capacity) {
        frameQueue = new EvictingBlockingQueue<>(new ArrayBlockingQueue<>(capacity));
        frameQueue.setEvictAction(Bitmap::recycle);
    }

    private void openCamera() {
        if (camera != null) return;
        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if (camera == null) {
            telemetry.addData("ERROR", "Camera not found or permission not granted");
            telemetry.update();
        }
    }

    private void startCamera() {
        if (cameraCaptureSession != null) {
            return;
        }

        final int imageFormat = ImageFormat.YUY2;
        CameraCharacteristics camCharacteristics = cameraName.getCameraCharacteristics();

        if (Arrays.stream(camCharacteristics.getAndroidFormats()).noneMatch(v -> v == imageFormat)) {
            telemetry.addData("ERROR: ", "image format not supported");
            telemetry.update();
            return;
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
                        session.startCapture(
                                captureRequest,
                                (session1, request, cameraFrame) -> {
                                    Bitmap bmp = captureRequest.createEmptyBitmap();
                                    cameraFrame.copyToBitmap(bmp);
                                    frameQueue.offer(bmp);
                                },
                                Continuation.create(callbackHandler, (session12, cameraCaptureSequenceId, lastFrameNumber) -> {
                                    RobotLog.ii("QR Test", "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                })
                        );
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
        }

        try {
            synchronizer.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        cameraCaptureSession = synchronizer.getValue();
    }

    private void stopCamera() {
        if (cameraCaptureSession == null) {
            return;
        }

        cameraCaptureSession.stopCapture();
        cameraCaptureSession.close();
        cameraCaptureSession = null;
    }

    private void closeCamera() {
        if (camera == null) {
            return;
        }

        stopCamera();
        camera.close();
        camera = null;
    }

    // -------------------
    //       Utils
    // -------------------
    public String readQRCode(Bitmap bitmap) {
        int[] pixelsArray = new int[bitmap.getWidth() * bitmap.getHeight()];
        bitmap.getPixels(pixelsArray, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());
        LuminanceSource source = new RGBLuminanceSource(bitmap.getWidth(), bitmap.getHeight(), pixelsArray);
        BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));

        try {
            return qrCodesReader.decode(binaryBitmap).getText();
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }
}