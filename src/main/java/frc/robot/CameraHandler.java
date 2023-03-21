package frc.robot;

import java.util.ArrayList;
import java.util.List;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;

public class CameraHandler {

    private List<VideoSource> cameras;
    private VideoSink sink;
    private int index;

    public CameraHandler(VideoSource... cams) {
        sink = CameraServer.addSwitchedCamera("POV: You are Itzik");
        cameras = new ArrayList<>();
        index = 0;

        for (var cam : cams) {
            cameras.add(cam);
            cam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        }

        setCamera(0);
    }

    public void addCameras(VideoSource... cameras) {
        for (var cam : cameras) {
            this.cameras.add(cam);
            cam.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        }
    }

    public int getIndex() {
        return index;
    }
    
    public VideoSource getActiveSource() {
        return sink.getSource();
    }

    public void setCamera(int index) {
        this.index = index;
        var currCam = sink.getSource();
        var putCam = cameras.get(index);
        // Make sure only one camera has "good" quality
        currCam.setVideoMode(PixelFormat.kGray, 4, 3, 1);
        putCam.setVideoMode(PixelFormat.kYUYV, 160, 120, 30);
        sink.setSource(putCam);
    }

    public void switchCamera() {
        setCamera((index + 1) % cameras.size());
    }

    public VideoSource getCamera(int index) {
        return cameras.get(index);
    }

}
