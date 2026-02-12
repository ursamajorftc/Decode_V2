package org.firstinspires.ftc.teamcode.subsystems;


import java.util.ArrayList;
import java.util.List;

public class TelemetryDebug {
    public ArrayList<watcher> watchers = new ArrayList<>();


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


