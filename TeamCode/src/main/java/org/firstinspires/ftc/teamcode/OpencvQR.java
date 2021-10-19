package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.graphics.Camera;
import android.graphics.ImageFormat;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraManager;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.collections.EvictingBlockingQueue;
import org.firstinspires.ftc.robotcore.internal.network.CallbackLooper;
import org.firstinspires.ftc.robotcore.internal.system.ContinuationSynchronizer;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;

import java.util.concurrent.TimeUnit;
import java.util.logging.Handler;

@Autonomous(name = "OpenCV QR")
public class OpencvQR extends LinearOpMode {
    private static final int secondsPermissonTimeout = Integer.MAX_VALUE;
    private CameraManager cameraManager;
    private WebcamName cameraName;
    private Camera camera;
    private CameraCaptureSession cameraCaptureSession;
    private EvictingBlockingQueue<Bitmap> frameQueue;

    private Handler callbackHandler;

    private Robot robot;

    @Override
    public void runOpMode() throws InterruptedException{

        callbackHandler = CallbackLooper.getDefault().getHandler();

        cameraManager = ClassFactory.getInstance().getCameraManager();
        cameraName = hardwareMap.get(WebcamName.class, "Webcam");

        robot = new Robot(hardwareMap);

        initFrameQueue(2);

        try{
            openCamera();
        } finally{
            closeCamera();
        }
    }

    private void onNewFrame(Bitmap frame){

    }

    private void initFrameQueue(int capacity){

    }

    public void openCamera(){
        if(camera != null){
            return ;
        }
        Deadline deadline = new Deadline(secondsPermissonTimeout, TimeUnit.SECONDS);
        camera = cameraManager.requestPermissionAndOpenCamera(deadline, cameraName, null);
        if(camera == null){
            telemetry.addData("ERROR", "camera not found or permission denied");
            telemetry.update();
        }
    }

    private void startCamera(){
        if(cameraCaptureSession != null){
            return ;
        }
        final int imageFormat = ImageFormat.YUY2;
        CameraCharacteristics cameraCharacteristics = cameraName.getCameraCharacteristics();
        if(!contains(cameraCharacteristics.getAndroidFormats(), imageFormat)){
            telemetry.addData("ERROR", "image format not supported");
            telemetry.update();
        }
        final Size size = cameraCharacteristics.getDefaultSize(imageFormat);
        final int fps = cameraCharacteristics.getMaxFramesPerSecond(imageFormat, size);
        final ContinuationSynchronizer<CameraCaptureSession> synchronizer = new ContinuationSynchronizer<>();

        try{
            camera.create
        }
    }

    private void stopCamera(){
        if(cameraCaptureSession != null){
            cameraCaptureSession.stopCapture();
            cameraCaptureSession.close();
            cameraCaptureSession = null;
        }
    }

    private void closeCamera(){
        stopCamera();
        if(camera != null){
            camera.close();
            camera = null;
        }
    }

    public String readQRCode(Bitmap bitmap){

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
