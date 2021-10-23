package org.firstinspires.ftc.teamcode;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.os.Handler;
import android.view.View;
import android.widget.LinearLayout;

import androidx.annotation.NonNull;

import com.google.zxing.BinaryBitmap;
import com.google.zxing.ChecksumException;
import com.google.zxing.EncodeHintType;
import com.google.zxing.FormatException;
import com.google.zxing.LuminanceSource;
import com.google.zxing.NotFoundException;
import com.google.zxing.RGBLuminanceSource;
import com.google.zxing.Reader;
import com.google.zxing.Result;
import com.google.zxing.ResultPoint;
import com.google.zxing.common.HybridBinarizer;
import com.google.zxing.qrcode.QRCodeReader;
import com.google.zxing.qrcode.decoder.ErrorCorrectionLevel;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
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
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.TimeUnit;

import me.dm7.barcodescanner.zxing.ZXingScannerView;

@Autonomous(name="QR Navigator")
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

    @Override
    public void runOpMode() throws InterruptedException {

        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        robot = new Robot(hardwareMap);

        gripper = hardwareMap.servo.get("gripper");
        gripper.setPosition(0.0);

        // cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        initFrameQueue(2);

        try {
            openCamera();
            if(camera == null) {
                telemetry.addData("ERROR: ", "cannot open camera");
                telemetry.update();
                return;
            }
            startCamera();
            if(cameraCaptureSession == null) {
                telemetry.addData("ERROR: ", "cannot start camera");
                telemetry.update();
                return ;
            }
            waitForStart();
            
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcamViewport = new FuckingViewPort(cameraMonitorViewId);

            telemetry.clear();
            telemetry.addData("> ", "Started");
            telemetry.update();

            while(true && opModeIsActive()) {
                if(gamepad1.a) gripper.setPosition(0.7);
                if(gamepad1.b) gripper.setPosition(0.3);

                Bitmap bmp = frameQueue.poll();

                if(bmp != null) {
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

        if(instruction != null) {
            telemetry.addData("Instruction", instruction);
            telemetry.addData("Turbo", robot.isTurbo());
            telemetry.update();

            double x = 0;
            double y = 0;
            double rotationValue = 0;

            switch (instruction){
                // in current circumstances, we are going to use case "2" as "forward" and case "1" as "backward"
                case "2": // backward
                    x = 0.0; y = -1.0; rotationValue = 0.0;
                    try{
                        final double speed = Math.min(1.0, Math.sqrt(x * x + y * y)); // reading from the controller values
                        final double rotation = Math.pow(rotationValue, 3.0);
                        final double direction = Math.atan2(x, y) - robot.getHeadingDegrees();

                        final double lf = speed * Math.sin(direction + Math.PI / 4.0) - rotation;
                        final double lr = speed * Math.cos(direction + Math.PI / 4.0) + rotation;
                        final double rf = speed * Math.cos(direction + Math.PI / 4.0) - rotation;
                        final double rr = speed * Math.sin(direction + Math.PI / 4.0) + rotation;

                        robot.setMotors(lf, lr, rf, rr, robot.isTurbo());
                        Thread.sleep(2000);
                    } catch(Exception e) { }
                    break;

                case "5":
                    try{
                        gripper.setPosition(1.0);
                        Thread.sleep(1000);
                        gripper.setPosition(0.0);
                    } catch(Exception e){ }
            }

            robot.stop();
        }
    }

    // -------------------
    //    Camera logic
    // -------------------
    private void initFrameQueue(int capacity) {
        frameQueue = new EvictingBlockingQueue<>(new ArrayBlockingQueue<>(capacity));
        frameQueue.setEvictAction(new Consumer<Bitmap>() {
            @Override
            public void accept(Bitmap frame) {
                frame.recycle();
            }
        });
    }
    private void openCamera() {
        if(camera != null) return ;
        Deadline deadline = new Deadline(secondsPermissionTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if(camera == null) {
            telemetry.addData("ERROR", "Camera not found or permission not granted");
            telemetry.update();
        }
    }
    private void startCamera() {
        if(cameraCaptureSession != null) return ;
        final int imageFormat = ImageFormat.YUY2;
        CameraCharacteristics camCharacteristics = cameraName.getCameraCharacteristics();
        if(!contains(camCharacteristics.getAndroidFormats(), imageFormat)) {
            telemetry.addData("ERROR: ", "image format not supported");
            telemetry.update();
        }
        final Size size = camCharacteristics.getDefaultSize(imageFormat);
        final int fps = camCharacteristics.getMaxFramesPerSecond(imageFormat, size);
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();
        try {
            camera.createCaptureSession(Continuation.create(callbackHandler, new CameraCaptureSession.StateCallbackDefault() {
                @Override public void onConfigured(@NonNull CameraCaptureSession session) {
                    try {
                        final CameraCaptureRequest captureRequest = camera.createCaptureRequest(imageFormat, size, fps);
                        session.startCapture(captureRequest,
                                new CameraCaptureSession.CaptureCallback() {
                                    @Override public void onNewFrame(@NonNull CameraCaptureSession session, @NonNull CameraCaptureRequest request, @NonNull CameraFrame cameraFrame) {
                                        Bitmap bmp = captureRequest.createEmptyBitmap();
                                        cameraFrame.copyToBitmap(bmp);
                                        frameQueue.offer(bmp);
                                    }
                                },
                                Continuation.create(callbackHandler, new CameraCaptureSession.StatusCallback() {
                                    @Override public void onCaptureSequenceCompleted(@NonNull CameraCaptureSession session, CameraCaptureSequenceId cameraCaptureSequenceId, long lastFrameNumber) {
                                        RobotLog.ii("QR Test", "capture sequence %s reports completed: lastFrame=%d", cameraCaptureSequenceId, lastFrameNumber);
                                    }
                                })
                        );
                        synchronizer.finish(session);
                    } catch (CameraException |RuntimeException e) {
                        RobotLog.ee("QR Test", e, "exception starting capture");
                        telemetry.addData("ERROR: ", "exception starting capture");
                        telemetry.update();
                        session.close();
                        synchronizer.finish(null);
                    }
                }
            }));
        } catch (CameraException|RuntimeException e) {
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
        if(cameraCaptureSession != null) {
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }
    private void closeCamera() {
        stopCamera();
        if(camera != null) {
            camera.close();
            camera = null;
        }
    }
    // -------------------
    //       Utils
    // -------------------
    public String readQRCode(Bitmap bitmap) {
        String decoded = null;
        int[] pixelsArray = new int[bitmap.getWidth() * bitmap.getHeight()];
        bitmap.getPixels(pixelsArray, 0, bitmap.getWidth(), 0, 0, bitmap.getWidth(), bitmap.getHeight());
        LuminanceSource source = new RGBLuminanceSource(bitmap.getWidth(), bitmap.getHeight(), pixelsArray);
        BinaryBitmap binaryBitmap = new BinaryBitmap(new HybridBinarizer(source));
        Reader reader = new QRCodeReader();
        try {
            Result result = reader.decode(binaryBitmap);
            decoded = result.getText();

            // ResultPoint[] resultPoints = result.getResultPoints();
        } catch (NotFoundException e) {
            e.printStackTrace();
        } catch (ChecksumException e) {
            e.printStackTrace();
        } catch (FormatException e) {
            e.printStackTrace();
        }
        
        return decoded;
    }
    private boolean contains(int[] array, int value) {
        for(int i : array) {
            if (i == value) return true;
        }
        return false;
    }
}
