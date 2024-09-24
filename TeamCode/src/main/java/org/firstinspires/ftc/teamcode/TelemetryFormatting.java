package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TelemetryFormatting {
    private LinearOpMode lOpMode;
    private org.firstinspires.ftc.robotcore.external.Telemetry t;

    public TelemetryFormatting(LinearOpMode l){
        lOpMode = l;
        t = lOpMode.telemetry;
    }

    public void addLineBreaks(int n) {
        for(int i = 0; i < n; i++){
            t.addLine();
        }
    }


}
