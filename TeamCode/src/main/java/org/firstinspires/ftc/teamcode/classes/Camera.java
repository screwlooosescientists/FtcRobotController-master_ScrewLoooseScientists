package org.firstinspires.ftc.teamcode.classes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import org.firstinspires.ftc.teamcode.classes.extra.Position;

import java.util.List;


public class Camera extends Robot{

    public CameraName cam;
    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    public TfodProcessor tfod;

    //varialbles Aprill tags
    List<AprilTagDetection> Aprildetectiones; // list with all detected tags
    AprilTagDetection Aprildetec;                  // current detected tag
    int AprildetecID;


    public Camera(CameraName cam, VisionPortal visionPortal, AprilTagProcessor aprilTag, TfodProcessor tfod)
    {
        this.cam = cam;
        this.visionPortal = visionPortal;
        this.aprilTag = aprilTag;
        this .tfod = tfod;

    }

    public void initVSProcesor() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();


        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName("vision.tflite")
                //.setModelFileName(TFOD_MODEL_FILE)

                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();


        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

            builder.setCamera(cam);


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));
        builder.setCameraResolution(new Size(1280, 720));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);
        builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    public void getTagids()
    {
        Aprildetectiones = aprilTag.getDetections();
        for (AprilTagDetection Aprildetec : Aprildetectiones) {
            AprildetecID = Aprildetec.id;
        }

    }


    public Recognition HighestRecon()
    {
        Recognition HighestRecon = null; //init the variable as empty
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        if(currentRecognitions.size() > 1)
        {
            for(int i = 0; i < currentRecognitions.size(); i ++)
            {
                if(currentRecognitions.get(i).getConfidence() > HighestRecon.getConfidence())
                {
                    HighestRecon = currentRecognitions.get(i);
                }
            }
        }
        else if (currentRecognitions.size() == 1)
        {
            HighestRecon = currentRecognitions.get(0); //if there is only one recognition it is always the highest
        }
        return HighestRecon;
    }

    public Position GetTfodLocation(Recognition recognition) // Gets the location of an particular recognition
    {
        if(recognition == null)
            return new Position(0, 0);

        double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
        double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;
        return new Position(x, y);
    }

    @Override
    public void Init() {


       initVSProcesor();
    }
}
