/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Utils;

import java.util.List;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.VideoCamera;
import edu.wpi.first.cameraserver.CameraServer;

/**
 * Add your docs here.
 */
public class HandleCameras {

    private Map<String, VideoCamera> cameras;
    private MjpegServer server;

    public HandleCameras(List<VideoCamera> cams, String toStream) {
        CameraServer camServerInst = CameraServer.getInstance();
        this.cameras = new HashMap<>();
        for (VideoCamera cam : cams) {
            String name = cam.getName();
            this.cameras.put(name, cam);
        }
        //Server-URL:http://roborio-team-frc.local:port/?action=stream
        //Therefore the url for the server below is:http://roborio-4586-frc.local:1234/?action=stream
        this.server = camServerInst.addServer("Handle Cameras Server", 5800); //name, port
        this.server.getProperty("compression").set(60);
        // this.server.getProperty("fps").set(30);
        // this.server.getProperty("width").set(320);
        // this.server.getProperty("height").set(240);
        this.setStreaming(toStream);
    }

    public HandleCameras(List<VideoCamera> cams) {
        this(cams, cams.get(0).getName());
    }

    public void setStreaming(String toStream) {
        if (this.cameras.containsKey(toStream)) {
            VideoCamera cam = this.cameras.get(toStream);
            this.server.setSource(cam);
        }
    }

    public String getStreaming() {
        return this.server.getSource().getName();
    }

    public Set<String> getNames() {
        return this.cameras.keySet();
    }
}
