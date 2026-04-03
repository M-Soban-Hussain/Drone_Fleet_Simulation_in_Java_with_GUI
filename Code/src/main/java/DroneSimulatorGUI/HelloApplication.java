package DroneSimulatorGUI;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.stage.Stage;

public class HelloApplication extends Application {

    @Override
    public void start(Stage stage) throws Exception {
        FXMLLoader fxmlLoader = new FXMLLoader(HelloApplication.class.getResource("hello-view.fxml"));
        Scene scene = new Scene(fxmlLoader.load());

        stage.setTitle("Drone Fleet Simulator");
        stage.setScene(scene);

        stage.setResizable(true);
        stage.setMinWidth(1100);
        stage.setMinHeight(700);

        stage.setWidth(1500);
        stage.setHeight(850);

        stage.show();
    }

    public static void main(String[] args) {
        launch();
    }
}
