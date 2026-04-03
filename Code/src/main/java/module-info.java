module DroneSimulatorGUI {
    requires javafx.controls;
    requires javafx.fxml;
    requires java.desktop;
    requires java.prefs;

    opens DroneSimulatorGUI to javafx.fxml;
    exports DroneSimulatorGUI;
}
