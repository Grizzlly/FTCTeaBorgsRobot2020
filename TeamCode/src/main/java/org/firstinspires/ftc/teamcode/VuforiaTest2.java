package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Vector;
@Autonomous(name="VuforiaTest2", group ="Concept")

public class VuforiaTest2 extends LinearOpMode
{
    private AutonomousFunctions func = new AutonomousFunctions();

    @Override
    public void runOpMode() throws InterruptedException
    {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.vuforiaLicenseKey = "AcE6+db/////AAABmW9N0TtrukmOuMZr+N/zDQ4KCGjDzsDdrQQqnECatVD9lks8RN3A0xkbMzdScVpGSw33XbQ0EbpkpzLZOYdzjRdvVw3ieASzbN6RkTktZI0C9rwSxWW8FzA8WCYXgVbj3m4dIG1MqTmRr8J3kH3S/+YpamUB65kfIkyM+QBjei3G1rqj0r2WtHFCRa4mqeEIIETpz5IMA2GFOH/vwrxHf8pAsAlwfMFbm94PryZK3ik609yEJ5WcV4j4MiEy0ThCllBLvSxfsatFdZLlhIjeJaZgbPAlE4MbmKRAPQQLdNturh1aZXEPdzEnvTt0UtXDZE2ow3AIKJe4VeGpDDLJmfaRZK7yKHdoIpGi8jir1Mw4";
        parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        parameters.useExtendedTracking = false;

        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(parameters);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        //All targets are loaded from "Skystone", under FtcRobotController->assets
        VuforiaTrackables allTargets = vuforia.loadTrackablesFromAsset("Skystone");

        //reference to the SkyStone and set its name to SkyStone
        VuforiaTrackable SkyStone = allTargets.get(0);
        SkyStone.setName("SkyStone");

        //waitForStart();

        allTargets.activate();

        func = new AutonomousFunctions();
        func.init(hardwareMap);

        while(opModeIsActive())
        {
            for(VuforiaTrackable target : allTargets)
            {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)target.getListener()).getUpdatedRobotLocation();

                if(pose != null)
                {
                    VectorF translation = pose.getTranslation();

                    if(translation.get(2) > 200)
                    {
                        func.Move_Forward(0.8);
                    }
                    else func.Stop_Robot();


                    telemetry.addData(target.getName() + "-Translation(X, Y, Z)", translation);

                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(2)));
                    telemetry.addData("Degrees To Turn: ", degreesToTurn);
                }else func.Stop_Robot();
            }
            telemetry.update();
        }
        allTargets.deactivate();
    }
}
