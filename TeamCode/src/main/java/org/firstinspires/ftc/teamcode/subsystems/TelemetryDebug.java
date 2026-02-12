package org.firstinspires.ftc.teamcode.subsystems;


import java.util.ArrayList;
import java.util.List;

public class TelemetryDebug {
    public List<watcher> watchers = new ArrayList<>();


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


