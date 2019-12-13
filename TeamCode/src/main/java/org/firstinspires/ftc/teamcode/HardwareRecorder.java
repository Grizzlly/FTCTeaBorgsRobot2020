package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Function;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.json.JSONArray;
//import org.json.JSONException;
//import org.json.JSONObject;
import org.json.JSONTokener;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.stream.JsonReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.io.PrintWriter;

public class HardwareRecorder
{
    private String name;
    private DcMotor[] dcMotors;

    private File file;
    private PrintWriter pw;

    private Gson gson;
    private JsonObject gsonobj;

    private boolean recording = false;

    private int ticks = 0;

    public HardwareRecorder(String name)
    {
        this.name=name;
    }


    //TODO: ADD OTHER TYPES OF MOTORS
    public void InitMotors(DcMotor ...objects)
    {
        dcMotors=objects;
    }

    public void StartRecording() throws IOException
    {
        File dir = Environment.getExternalStorageDirectory();
        file = new File(dir, "testf.json");
        file.createNewFile();
        file.setWritable(true);
        file.setReadable(true);

        pw = new PrintWriter(file);

        gsonobj = new JsonObject();

        ticks = 0;
        recording = true;
    }

    public void StopRecording()
    {
        if(recording){
            //jo.put("TotalTicks", ticks);
            gsonobj.addProperty("TotalTicks", ticks);
            pw.print(gsonobj.toString());

            pw.flush();
            pw.close();

            recording = false;}
    }

    public void Step(double time)
    {
        if(recording)
        {
            //JSONArray ja = new JSONArray();
            JsonArray jag = new JsonArray();


            for(int i=0; i<dcMotors.length; i++)
            {
                jag.add(dcMotors[i].getPower());
            }

            //for (DcMotor motor : dcMotors)
            //{
            //    jag.add(motor.getPower());
            //ja.put(motor.getPower());
            //}

            jag.add(time);



            //jo.put(String.valueOf(ticks), ja);
            gsonobj.add(String.valueOf(ticks), jag);
        }
        ticks++;
    }

    public void Play(Telemetry telemetry) throws FileNotFoundException
    {

        File dir = Environment.getExternalStorageDirectory();
        file = new File(dir, "testf.json");

        //Object object = new JsonParser().parse(file.getAbsolutePath());
        //JsonReader gsonr = new JsonReader(new FileReader(file));

        JsonParser jsonParser = new JsonParser();

        JsonObject jsonObject = jsonParser.parse(new FileReader(file)).getAsJsonObject();


        try
        {
            //DcMotorReader reader = gson.fromJson(gsonr, DcMotorReader.class);



            int totalticks = jsonObject.get("TotalTicks").getAsInt();
            telemetry.addLine(String.valueOf(totalticks));telemetry.update();


            for(int i=0; i<totalticks; i++)
            {
                JsonArray arr = jsonObject.getAsJsonArray(String.valueOf(i));
                //telemetry.addData("Array ", arr.toString());
                //telemetry.addData("ArraySize", arr.size());
                //telemetry.update();

                for(int j=0; j<arr.size()-1; j++)
                {
                    dcMotors[j].setPower(arr.get(j).getAsDouble());
                    //telemetry.addData("Motor"+j, dcMotors[j].getPower());

                    //telemetry.update();

                }
                double toRunTime = arr.get(arr.size()-1).getAsLong();
                //telemetry.addData("RunTime", toRunTime);
                //telemetry.update();
                Thread.sleep((long)toRunTime);

                //while(runtime.time()<toRunTime)
                //{
                    //telemetry.addLine("Executing...");
                    //telemetry.update();
               // }
            }

        }catch(Exception e){telemetry.addLine(e.getMessage());telemetry.update();}

    }
    //TODO: FINISH!!!
}
