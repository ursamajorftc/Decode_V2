package org.firstinspires.ftc.teamcode.subsystems;


import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;

public class TelemetryDebug {
    public static ArrayList<watcher> watchers;
    public static Telemetry telemetry;

    public TelemetryDebug () {
        watchers = new ArrayList<>();
    }
    public TelemetryDebug (Telemetry telemetry) {
        watchers = new ArrayList<>();
        TelemetryDebug.telemetry = telemetry;
    }

    public void createWatcher(String name, Object value){
        boolean exists = false;
        if (!watchers.isEmpty())
            for (watcher w : watchers){
                if (w.getName().equals(name)){
                    w.value = value;
                    exists = true;
                    break;
                }
            }
        if(!exists){
            watchers.add(new watcher(name, value));
        }
    }

    public void createWatcher(String name, double value){
        Double valueObject = value;
        boolean exists = false;
        if (!watchers.isEmpty())
            for (watcher w : watchers){
                if (w.getName().equals(name)){
                    w.value = valueObject;
                    exists = true;
                    break;
                }
            }
        if(!exists){
            watchers.add(new watcher(name, valueObject));
        }
    }

    public void createWatcher(String name, int value){
        Integer valueObject = value;
        boolean exists = false;
        if (!watchers.isEmpty())
            for (watcher w : watchers){
                if (w.getName().equals(name)){
                    w.value = valueObject;
                    exists = true;
                    break;
                }
            }
        if(!exists){
            watchers.add(new watcher(name, valueObject));
        }
    }

    public void createWatcher(String name, boolean value){
        Boolean valueObject = value;
        boolean exists = false;
        if (!watchers.isEmpty())
            for (watcher w : watchers){
                if (w.getName().equals(name)){
                    w.value = valueObject;
                    exists = true;
                    break;
                }
            }
        if(!exists){
            watchers.add(new watcher(name, valueObject));
        }
    }

    public void logData () {
        if (telemetry != null) {
            for (watcher w : watchers) {
                telemetry.addData(w.getName(), w.getValue());
            }
        } else {
            return;
        }
    }

    public static class watcher<T> {
        private String name;
        private T value;
        public watcher(String name, T value){
            this.name = name;
            this.value = value;
        }
        public String getName () {return name;}
        public T getValue () {return value;}
    }

}


