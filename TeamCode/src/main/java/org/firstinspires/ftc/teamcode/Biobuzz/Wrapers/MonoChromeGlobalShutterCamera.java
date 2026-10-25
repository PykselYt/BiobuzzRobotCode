package org.firstinspires.ftc.teamcode.Biobuzz.Wrapers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class MonoChromeGlobalShutterCamera {

    OpenCvWebcam webcam;

    public MonoChromeGlobalShutterCamera(String name, HardwareMap hardwaremap){
        int cameraMonitorViewId = hardwaremap.appContext
                .getResources().getIdentifier("cameraMonitorViewId", "id", hardwaremap.appContext.getPackageName());

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwaremap.get(WebcamName.class, name), cameraMonitorViewId);

    }

    public void startCamera() {
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
                // ------------------ Tzeapa frate
            }

        });
        FtcDashboard.getInstance().startCameraStream(webcam,0);
    }

    public void ShutDownCamera(){
        webcam.closeCameraDeviceAsync(new OpenCvCamera.AsyncCameraCloseListener() {
            @Override
            public void onClose() {
                //closed camera
            }
        });
    }

}
