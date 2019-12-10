package org.firstinspires.ftc.teamcode;

import org.json.JSONArray;
//import org.json.JSONException;
//import org.json.JSONObject;
import org.json.JSONTokener;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    public void StartRecording() throws FileNotFoundException, IOException
    {
        file = new File(name+".json");
        file.createNewFile();

        pw = new PrintWriter(file);

        ticks = 0;
        recording = true;
    }

    public void StopRecording()
    {
        //jo.put("TotalTicks", ticks);
        gsonobj.addProperty("TotalTicks", ticks);
        pw.print(gsonobj.toString());

        pw.flush();
        pw.close();

        recording = false;
    }

    public void Step()
    {
        if(recording)
        {
            //JSONArray ja = new JSONArray();
            JsonArray jag = new JsonArray();

            for (DcMotor motor : dcMotors)
            {
                jag.add(motor.getPower());
                //ja.put(motor.getPower());
            }

            //jo.put(String.valueOf(ticks), ja);
            gsonobj.add(String.valueOf(ticks), jag);
        }
    }

    public void Play() throws FileNotFoundException
    {
        Object object = new JsonParser().parse(new FileReader(name));

        JsonObject gson = (JsonObject) object;

    }
    //TODO: FINISH!!!
}
