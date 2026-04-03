package DroneSimulatorGUI;

import DroneSimulator.*;
import javafx.animation.AnimationTimer;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.Scene;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.*;
import javafx.scene.image.*;
import javafx.scene.input.KeyCode;
import javafx.scene.input.MouseButton;
import javafx.scene.input.ScrollEvent;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.scene.text.Font;
import javafx.stage.FileChooser;

import java.io.*;
import java.util.*;

public class DroneSimulatorController {

    // ===== Tabs root =====
    @FXML private TabPane rootTabs;

    // ===== SIM (Center) =====
    @FXML private StackPane sceneContainer;
    @FXML private Canvas mapCanvas;

    // ===== SIM (Left) =====
    @FXML private ComboBox<String> droneStyleCombo;
    @FXML private ComboBox<String> bgStyleCombo;
    @FXML private TextField customDroneField;
    @FXML private TextField customBgField;
    @FXML private Button browseCustomDroneBtn;
    @FXML private Button browseCustomBgBtn;
    @FXML private Button applyVisualsBtn;
    @FXML private CheckBox removeWhiteCheck;
    @FXML private Slider whiteToleranceSlider;
    @FXML private Slider tiltSlider;
    @FXML private CheckBox gridCheck;
    @FXML private CheckBox labelsCheck;
    @FXML private CheckBox linksCheck;
    @FXML private CheckBox sprayCheck;
    @FXML private Label followLabel;

    // ===== SIM (Right) =====
    @FXML private TextField droneCountField;
    @FXML private TextField totalTimeField;
    @FXML private Button loadConstantsBtn;

    @FXML private Button initDronesRandomBtn;
    @FXML private Button initDronesZeroBtn;

    @FXML private Button randomTargetBtn;
    @FXML private TextField targetXField;
    @FXML private TextField targetYField;
    @FXML private TextField targetZField;
    @FXML private Button setTargetBtn;

    @FXML private Button randomTargetsPerDroneBtn;
    @FXML private TextField droneIdField;
    @FXML private TextField droneTargetXField;
    @FXML private TextField droneTargetYField;
    @FXML private TextField droneTargetZField;
    @FXML private Button setDroneTargetBtn;

    @FXML private Button startBtn;
    @FXML private Button pauseBtn;
    @FXML private Button resetBtn;
    @FXML private Slider speedSlider;
    @FXML private Label targetLabel;
    @FXML private Label statusLabel;

    // ===== GAME TAB =====
    @FXML private StackPane gameSceneContainer;
    @FXML private Canvas gameCanvas;
    @FXML private Button gameStartBtn;
    @FXML private Button gameRandomTargetBtn;
    @FXML private Button gameResetBtn;
    @FXML private Label gameInfoLabel;

    @FXML private ComboBox<String> gameDroneStyleCombo;
    @FXML private ComboBox<String> gameBgStyleCombo;
    @FXML private TextField gameCustomDroneField;
    @FXML private TextField gameCustomBgField;
    @FXML private Button gameBrowseDroneBtn;
    @FXML private Button gameBrowseBgBtn;
    @FXML private Button gameApplyVisualsBtn;

    // ===================== Simulation parameters (physics defaults) =====================
    private double dt = 0.01;
    private double totalTime = 60.0;
    private int numDrones = 4;

    private double mass = 1.5;
    private double kd = 0.2;
    private double Ix = 0.02, Iy = 0.02, Iz = 0.04;
    private double kp = 3.5;
    private double kdCtrl = 2.5;
    private double kR = 8.5;
    private double kOmega = 0.15;

    private double formKp = 0.0;
    private double formKv = 0.0;
    private double kRep = 1.0;
    private double commRange = 50.0;
    private double packetLoss = 0.01;

    private double envWidth = 100.0;
    private double envHeight = 100.0;
    private double envDepth = 1000.0;
    private double minSafeDistance = 1.0;

    private String positionsCsv = "Positions.csv";
    private String metricsTxt = "Metrics.txt";

    // ===================== Core sim objects =====================
    private final Random rand = new Random();
    private List<Drone> drones = new ArrayList<>();
    private Simulator simulator;
    private Logger logger;

    private boolean running = false;
    private boolean finished = false;
    private boolean metricsWritten = false;
    private double simTime = 0.0;
    private long lastNs = 0;
    private double accumulator = 0;
    private AnimationTimer loop;

    // ===================== Targets =====================
    private Vector3 mainTarget = null;
    private boolean perDroneTargetsMode = false;
    private double formationRadius = 0.0;

    // ===================== Visuals (Simulation) =====================
    private String currentDroneStyle = "🚁 Quadcopter";
    private String currentBgStyle = "🌾 Farm Field";
    private Image customDroneImage = null;
    private Image customBgImage = null;

    // ===================== Visuals (Game) =====================
    private String gameDroneStyle = "🚁 Quadcopter";
    private String gameBgStyle = "🌾 Farm Field";
    private Image gameDroneImage = null;
    private Image gameBgImage = null;

    // ===================== Camera/view (2D) =====================
    private double zoom = 6.5;
    private double panX = 40;
    private double panY = 40;
    private double dragStartX, dragStartY;
    private double panStartX, panStartY;

    // ===================== Follow camera =====================
    private Integer followedDroneId = null;  // null = OFF
    private boolean followEnabled = false;
    private double followLerp = 0.10;        // smoothing

    // ===================== GAME state =====================
    private boolean gameRunning = false;
    private boolean gameMissionCompleted = false;

    private Vector3 gameTarget = null;
    private double gameTargetRadius = 10.0;  // BIGGER radius (world units)

    private Vector3 gamePos = new Vector3(10, 10, 2);
    private Vector3 gameVel = Vector3.zero();

    private double manualAccel = 28.0;      // responsiveness
    private double manualDrag = 6.0;

    private double sprayHoldSeconds = 0.0;
    private double missionSprayRequired = 10.0;

    private final Set<KeyCode> keysDown = new HashSet<>();

    // fireworks particles
    private static class Firework {
        double x,y, vx,vy, life;
        Color c;
        Firework(double x,double y,double vx,double vy,double life, Color c){
            this.x=x; this.y=y; this.vx=vx; this.vy=vy; this.life=life; this.c=c;
        }
    }
    private final List<Firework> fireworks = new ArrayList<>();
    private boolean victoryPopupShown = false;

    @FXML
    public void initialize() {
        // ===== style lists (same for sim + game) =====
        List<String> droneSkins = Arrays.asList(
                "🚁 Quadcopter","🛸 UFO","👻 Ghost","🦜 Parrot","🚀 Rocket","⚡ Lightning",
                "🐝 Bee","🐉 Dragon","🛰️ Satellite","🦇 Bat","🐙 OctoDrone"
        );
        List<String> bgSkins = Arrays.asList(
                "🌾 Farm Field","☁️ Sky","🏜️ Desert","🌊 Ocean","🌃 City Night","🌌 Space"
        );

        if (droneStyleCombo != null) {
            droneStyleCombo.getItems().setAll(droneSkins);
            droneStyleCombo.getSelectionModel().select("🚁 Quadcopter");
        }
        if (bgStyleCombo != null) {
            bgStyleCombo.getItems().setAll(bgSkins);
            bgStyleCombo.getSelectionModel().select("🌾 Farm Field");
        }

        if (gameDroneStyleCombo != null) {
            gameDroneStyleCombo.getItems().setAll(droneSkins);
            gameDroneStyleCombo.getSelectionModel().select("🚁 Quadcopter");
        }
        if (gameBgStyleCombo != null) {
            gameBgStyleCombo.getItems().setAll(bgSkins);
            gameBgStyleCombo.getSelectionModel().select("🌾 Farm Field");
        }

        if (speedSlider != null) {
            speedSlider.setMin(0.2);
            speedSlider.setMax(5.0);
            speedSlider.setValue(1.0);
        }
        if (tiltSlider != null) tiltSlider.setValue(0.55);

        if (totalTimeField != null) totalTimeField.setText(String.valueOf(totalTime));
        if (droneCountField != null) droneCountField.setText(String.valueOf(numDrones));

        if (removeWhiteCheck != null) removeWhiteCheck.setSelected(true);
        if (whiteToleranceSlider != null) whiteToleranceSlider.setValue(0.22);

        setupSimCanvasBindings();
        setupGameCanvasBindings();
        setupMouseControls();
        setupClickToFollow();
        setupKeyboardHandlersWhenSceneReady();

        // IMPORTANT: re-render when tilt changes (makes slider feel "working")
        if (tiltSlider != null) tiltSlider.valueProperty().addListener((o,a,b) -> renderSim());
        if (gridCheck != null) gridCheck.selectedProperty().addListener((o,a,b) -> renderSim());
        if (labelsCheck != null) labelsCheck.selectedProperty().addListener((o,a,b) -> renderSim());
        if (linksCheck != null) linksCheck.selectedProperty().addListener((o,a,b) -> renderSim());
        if (sprayCheck != null) sprayCheck.selectedProperty().addListener((o,a,b) -> renderSim());

        rebuildSimulator();
        setupLoop();
        updateTargetLabel();
        updateFollowLabel();

        if (statusLabel != null) statusLabel.setText("Ready. Initialize drones and set targets.");
        if (gameInfoLabel != null) gameInfoLabel.setText("Status: Ready (press Start Mission)");

        // Focus fixes for SPACE in Game tab
        Platform.runLater(() -> {
            if (rootTabs != null) {
                rootTabs.getSelectionModel().selectedItemProperty().addListener((obs, oldTab, newTab) -> {
                    if (newTab != null && "Game".equalsIgnoreCase(newTab.getText())) {
                        if (gameCanvas != null) gameCanvas.requestFocus();
                    } else {
                        if (mapCanvas != null) mapCanvas.requestFocus();
                    }
                });
            }
            if (mapCanvas != null) mapCanvas.requestFocus();
        });

        renderSim();
        renderGame();
    }

    // ===================== CANVAS BINDINGS =====================
    private void setupSimCanvasBindings() {
        if (sceneContainer == null || mapCanvas == null) return;
        mapCanvas.widthProperty().bind(sceneContainer.widthProperty());
        mapCanvas.heightProperty().bind(sceneContainer.heightProperty());
        mapCanvas.widthProperty().addListener((obs, o, n) -> renderSim());
        mapCanvas.heightProperty().addListener((obs, o, n) -> renderSim());
        mapCanvas.setFocusTraversable(true);
    }

    private void setupGameCanvasBindings() {
        if (gameSceneContainer == null || gameCanvas == null) return;
        gameCanvas.widthProperty().bind(gameSceneContainer.widthProperty());
        gameCanvas.heightProperty().bind(gameSceneContainer.heightProperty());
        gameCanvas.widthProperty().addListener((obs, o, n) -> renderGame());
        gameCanvas.heightProperty().addListener((obs, o, n) -> renderGame());
        gameCanvas.setFocusTraversable(true);
    }

    // ===================== INPUT: MOUSE (PAN/ZOOM) =====================
    private void setupMouseControls() {
        if (mapCanvas == null) return;

        mapCanvas.setOnMousePressed(e -> {
            if (e.getButton() == MouseButton.PRIMARY) {
                dragStartX = e.getX();
                dragStartY = e.getY();
                panStartX = panX;
                panStartY = panY;
            }
        });

        mapCanvas.setOnMouseDragged(e -> {
            if (e.getButton() == MouseButton.PRIMARY && !followEnabled) {
                double dx = e.getX() - dragStartX;
                double dy = e.getY() - dragStartY;
                panX = panStartX + dx;
                panY = panStartY + dy;
                renderSim();
            }
        });

        mapCanvas.addEventFilter(ScrollEvent.SCROLL, e -> {
            double oldZoom = zoom;
            double factor = (e.getDeltaY() > 0) ? 1.08 : 1.0 / 1.08;
            zoom = clamp(zoom * factor, 1.5, 25.0);

            double mx = e.getX();
            double my = e.getY();
            double wxBefore = screenToWorldX(mx);
            double wyBefore = screenToWorldY(my);

            double sxAfter = worldToScreenX(wxBefore);
            double syAfter = worldToScreenY(wyBefore);
            panX += (mx - sxAfter);
            panY += (my - syAfter);

            if (Math.abs(zoom - oldZoom) > 1e-6) renderSim();
            e.consume();
        });
    }

    // ===================== INPUT: CLICK-TO-FOLLOW =====================
    private void setupClickToFollow() {
        if (mapCanvas == null) return;

        mapCanvas.setOnMouseClicked(e -> {
            if (e.getButton() != MouseButton.PRIMARY) return;

            if (e.isControlDown()) {
                followEnabled = false;
                followedDroneId = null;
                updateFollowLabel();
                return;
            }

            if (drones == null || drones.isEmpty()) return;

            double mx = e.getX();
            double my = e.getY();

            int bestId = -1;
            double bestDist = 1e18;

            for (Drone d : drones) {
                Vector3 p = d.getPosition();
                double sx = worldToScreenX(p.x);
                double sy = worldToScreenY(p.y);

                // match render tilt
                double tilt = (tiltSlider != null) ? tiltSlider.getValue() : 0.55;
                sy -= (p.z * tilt * zoom * 0.035);

                double dx = mx - sx;
                double dy = my - sy;
                double dist2 = dx*dx + dy*dy;
                if (dist2 < bestDist) { bestDist = dist2; bestId = d.getId(); }
            }

            double clickRadius = 28.0;
            if (bestId >= 0 && bestDist <= clickRadius * clickRadius) {
                followedDroneId = bestId;
                followEnabled = true;
                updateFollowLabel();
            } else {
                followEnabled = false;
                followedDroneId = null;
                updateFollowLabel();
            }
        });
    }

    private void updateFollowLabel() {
        String text = (followEnabled && followedDroneId != null) ? ("Follow: Drone " + followedDroneId) : "Follow: OFF";
        if (followLabel != null) followLabel.setText(text);
    }

    // ===================== KEYBOARD HANDLERS =====================
    private void setupKeyboardHandlersWhenSceneReady() {
        if (sceneContainer != null) {
            sceneContainer.sceneProperty().addListener((obs, oldScene, newScene) -> {
                if (newScene != null) attachKeyHandlers(newScene);
            });
        }
        if (gameSceneContainer != null) {
            gameSceneContainer.sceneProperty().addListener((obs, oldScene, newScene) -> {
                if (newScene != null) attachKeyHandlers(newScene);
            });
        }
    }

    private void attachKeyHandlers(Scene scene) {
        // Don’t overwrite if already set by your app, but for your project this is fine:
        scene.setOnKeyPressed(e -> keysDown.add(e.getCode()));
        scene.setOnKeyReleased(e -> keysDown.remove(e.getCode()));
    }

    // ===================== CAMERA =====================
    @FXML
    public void resetCamera() {
        zoom = 6.5;
        panX = 40;
        panY = 40;
        followEnabled = false;
        followedDroneId = null;
        updateFollowLabel();
        renderSim();
    }

    private void followCameraStep() {
        if (!followEnabled || followedDroneId == null || drones == null || mapCanvas == null) return;

        Drone target = null;
        for (Drone d : drones) if (d.getId() == followedDroneId) { target = d; break; }
        if (target == null) {
            followEnabled = false;
            followedDroneId = null;
            updateFollowLabel();
            return;
        }

        double W = mapCanvas.getWidth();
        double H = mapCanvas.getHeight();

        Vector3 p = target.getPosition();
        double desiredPanX = (W * 0.5) - p.x * zoom;
        double desiredPanY = (H * 0.5) - p.y * zoom;

        panX = panX + (desiredPanX - panX) * followLerp;
        panY = panY + (desiredPanY - panY) * followLerp;
    }

    // ===================== FILE PICKERS (SIM) =====================
    @FXML
    public void browseCustomDrone() {
        FileChooser fc = imageChooser("Select Drone Image");
        File f = showChooser(fc);
        if (f != null && customDroneField != null) customDroneField.setText(f.getAbsolutePath());
    }

    @FXML
    public void browseCustomBackground() {
        FileChooser fc = imageChooser("Select Background Image");
        File f = showChooser(fc);
        if (f != null && customBgField != null) customBgField.setText(f.getAbsolutePath());
    }

    // ===================== FILE PICKERS (GAME) =====================
    @FXML
    public void browseGameDrone() {
        FileChooser fc = imageChooser("Select Game Drone Image");
        File f = showChooser(fc);
        if (f != null && gameCustomDroneField != null) gameCustomDroneField.setText(f.getAbsolutePath());
    }

    @FXML
    public void browseGameBackground() {
        FileChooser fc = imageChooser("Select Game Background Image");
        File f = showChooser(fc);
        if (f != null && gameCustomBgField != null) gameCustomBgField.setText(f.getAbsolutePath());
    }

    private FileChooser imageChooser(String title) {
        FileChooser fc = new FileChooser();
        fc.setTitle(title);
        fc.getExtensionFilters().addAll(
                new FileChooser.ExtensionFilter("Images", "*.png", "*.jpg", "*.jpeg", "*.gif"),
                new FileChooser.ExtensionFilter("All files", "*.*")
        );
        return fc;
    }

    private File showChooser(FileChooser fc) {
        if (sceneContainer != null && sceneContainer.getScene() != null) {
            return fc.showOpenDialog(sceneContainer.getScene().getWindow());
        }
        if (gameSceneContainer != null && gameSceneContainer.getScene() != null) {
            return fc.showOpenDialog(gameSceneContainer.getScene().getWindow());
        }
        return fc.showOpenDialog(null);
    }

    // ===================== VISUAL APPLY (SIM) =====================
    @FXML
    public void applyVisuals() {
        currentDroneStyle = (droneStyleCombo != null && droneStyleCombo.getValue() != null) ? droneStyleCombo.getValue() : currentDroneStyle;
        currentBgStyle = (bgStyleCombo != null && bgStyleCombo.getValue() != null) ? bgStyleCombo.getValue() : currentBgStyle;

        customDroneImage = null;
        customBgImage = null;

        String bgPath = (customBgField != null) ? customBgField.getText().trim() : "";
        if (!bgPath.isEmpty()) {
            try { customBgImage = new Image(new FileInputStream(bgPath)); }
            catch (Exception ex) { if (statusLabel != null) statusLabel.setText("Background load failed: " + ex.getMessage()); }
        }

        String dronePath = (customDroneField != null) ? customDroneField.getText().trim() : "";
        if (!dronePath.isEmpty()) {
            try {
                Image raw = new Image(new FileInputStream(dronePath));
                if (removeWhiteCheck != null && removeWhiteCheck.isSelected()) {
                    double tol = (whiteToleranceSlider != null) ? whiteToleranceSlider.getValue() : 0.22;
                    customDroneImage = chromaKeyRemoveNearWhite(raw, tol);
                } else customDroneImage = raw;
            } catch (Exception ex) {
                if (statusLabel != null) statusLabel.setText("Drone image load failed: " + ex.getMessage());
            }
        }

        if (statusLabel != null) statusLabel.setText("✓ Visuals applied");
        renderSim();
    }

    // ===================== VISUAL APPLY (GAME) =====================
    @FXML
    public void applyGameVisuals() {
        gameDroneStyle = (gameDroneStyleCombo != null && gameDroneStyleCombo.getValue() != null) ? gameDroneStyleCombo.getValue() : gameDroneStyle;
        gameBgStyle = (gameBgStyleCombo != null && gameBgStyleCombo.getValue() != null) ? gameBgStyleCombo.getValue() : gameBgStyle;

        gameDroneImage = null;
        gameBgImage = null;

        String bgPath = (gameCustomBgField != null) ? gameCustomBgField.getText().trim() : "";
        if (!bgPath.isEmpty()) {
            try { gameBgImage = new Image(new FileInputStream(bgPath)); }
            catch (Exception ex) { if (gameInfoLabel != null) gameInfoLabel.setText("Game BG load failed: " + ex.getMessage()); }
        }

        String dronePath = (gameCustomDroneField != null) ? gameCustomDroneField.getText().trim() : "";
        if (!dronePath.isEmpty()) {
            try {
                Image raw = new Image(new FileInputStream(dronePath));
                // reuse same white-removal toggle/slider for simplicity
                if (removeWhiteCheck != null && removeWhiteCheck.isSelected()) {
                    double tol = (whiteToleranceSlider != null) ? whiteToleranceSlider.getValue() : 0.22;
                    gameDroneImage = chromaKeyRemoveNearWhite(raw, tol);
                } else gameDroneImage = raw;
            } catch (Exception ex) {
                if (gameInfoLabel != null) gameInfoLabel.setText("Game Drone load failed: " + ex.getMessage());
            }
        }

        if (gameInfoLabel != null) gameInfoLabel.setText("✓ Game visuals applied");
        renderGame();
    }

    private Image chromaKeyRemoveNearWhite(Image src, double tolerance) {
        int w = (int) src.getWidth();
        int h = (int) src.getHeight();
        PixelReader pr = src.getPixelReader();
        WritableImage out = new WritableImage(w, h);
        PixelWriter pw = out.getPixelWriter();

        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                Color c = pr.getColor(x, y);
                double dr = 1.0 - c.getRed();
                double dg = 1.0 - c.getGreen();
                double db = 1.0 - c.getBlue();
                double dist = Math.sqrt(dr*dr + dg*dg + db*db);
                if (dist < tolerance) pw.setColor(x, y, Color.color(c.getRed(), c.getGreen(), c.getBlue(), 0.0));
                else pw.setColor(x, y, c);
            }
        }
        return out;
    }

    // ===================== LOAD CONSTANTS (FIXED) =====================
    @FXML
    public void loadConstantsFromFile() {
        FileChooser fc = new FileChooser();
        fc.setTitle("Load Constants (.properties or .txt)");
        fc.getExtensionFilters().addAll(
                new FileChooser.ExtensionFilter("Properties", "*.properties", "*.txt"),
                new FileChooser.ExtensionFilter("All files", "*.*")
        );
        File f = showChooser(fc);
        if (f == null) return;

        Properties props = new Properties();
        try (FileInputStream fis = new FileInputStream(f)) {
            props.load(fis);
        } catch (Exception ex) {
            if (statusLabel != null) statusLabel.setText("Constants load failed: " + ex.getMessage());
            return;
        }

        // helper
        dt = getD(props, "dt", dt);
        totalTime = getD(props, "totalTime", totalTime);
        numDrones = (int) getD(props, "numDrones", numDrones);

        mass = getD(props, "mass", mass);
        kd = getD(props, "kd", kd);

        Ix = getD(props, "Ix", Ix);
        Iy = getD(props, "Iy", Iy);
        Iz = getD(props, "Iz", Iz);

        kp = getD(props, "kp", kp);
        kdCtrl = getD(props, "kdCtrl", kdCtrl);
        kR = getD(props, "kR", kR);
        kOmega = getD(props, "kOmega", kOmega);

        formKp = getD(props, "formKp", formKp);
        formKv = getD(props, "formKv", formKv);

        kRep = getD(props, "kRep", kRep);
        commRange = getD(props, "commRange", commRange);
        packetLoss = getD(props, "packetLoss", packetLoss);

        envWidth = getD(props, "envWidth", envWidth);
        envHeight = getD(props, "envHeight", envHeight);
        envDepth = getD(props, "envDepth", envDepth);
        minSafeDistance = getD(props, "minSafeDistance", minSafeDistance);

        // reflect in UI
        if (totalTimeField != null) totalTimeField.setText(String.valueOf(totalTime));
        if (droneCountField != null) droneCountField.setText(String.valueOf(numDrones));

        // rebuild sim (keeps same physics logic, just new values)
        rebuildSimulator();

        if (statusLabel != null) statusLabel.setText("✓ Constants loaded: " + f.getName());
    }

    private double getD(Properties p, String key, double fallback) {
        String v = p.getProperty(key);
        if (v == null) return fallback;
        try { return Double.parseDouble(v.trim()); }
        catch (Exception ignored) { return fallback; }
    }

    // ===================== SIM CONFIG =====================
    @FXML
    public void applyTotalTime() {
        try {
            double t = Double.parseDouble(totalTimeField.getText().trim());
            if (t <= 0) { if (statusLabel != null) statusLabel.setText("Total time must be > 0."); return; }
            totalTime = t;
            if (statusLabel != null) statusLabel.setText("Total time set to " + totalTime + "s");
        } catch (Exception e) {
            if (statusLabel != null) statusLabel.setText("Invalid total time input.");
        }
    }

    @FXML
    public void applyDroneCount() {
        int n = safeParseInt((droneCountField != null ? droneCountField.getText() : ""), numDrones);
        n = (int) clamp(n, 1, 200);
        numDrones = n;

        resetAllStateHard();
        rebuildSimulator();
        updateTargetLabel();

        if (statusLabel != null) statusLabel.setText("Drone count set: " + numDrones + ". Initialize drones.");
        renderSim();
    }

    // ===================== INIT DRONES =====================
    @FXML
    public void initDronesRandom() {
        initDrones(false);
        if (statusLabel != null) statusLabel.setText("Drones initialized randomly. Set targets.");
        renderSim();
    }

    @FXML
    public void initDronesZero() {
        initDrones(true);
        if (statusLabel != null) statusLabel.setText("Drones initialized at origin. Set targets.");
        renderSim();
    }

    private void initDrones(boolean zeroStart) {
        finished = false;
        running = false;
        simTime = 0;
        accumulator = 0;
        lastNs = 0;
        metricsWritten = false;

        drones = new ArrayList<>();

        Vector3 inertia = new Vector3(Ix, Iy, Iz);
        Vector3 initialVelocity = Vector3.zero();
        Matrix3D initialRotation = Matrix3D.identity();
        Vector3 dummyTarget = new Vector3(0, 0, 0);
        Vector3 targetVelocity = Vector3.zero();

        for (int i = 0; i < numDrones; i++) {
            Vector3 initialPosition = zeroStart
                    ? new Vector3(0, 0, 0)
                    : new Vector3(rand.nextDouble() * 20, rand.nextDouble() * 20, 2.0 + rand.nextDouble() * 2.0);

            drones.add(new Drone(i, mass, inertia, kd, initialPosition, dummyTarget,
                    targetVelocity, initialVelocity, initialRotation));
        }

        mainTarget = null;
        formationRadius = 0.0;
        perDroneTargetsMode = false;

        followEnabled = false;
        followedDroneId = null;
        updateFollowLabel();

        rebuildSimulator();
        updateTargetLabel();
    }

    // ===================== TARGETS =====================
    @FXML
    public void randomizeTarget() {
        if (drones.isEmpty()) { if (statusLabel != null) statusLabel.setText("Initialize drones first."); return; }
        double x = rand.nextDouble() * 80 + 10;
        double y = rand.nextDouble() * 80 + 10;
        double z = Math.min(envDepth, rand.nextDouble() * 30 + 5);
        setMainTarget(new Vector3(x, y, z));
        if (statusLabel != null) statusLabel.setText("Random shared target set. Press Start.");
        renderSim();
    }

    @FXML
    public void setTarget() {
        if (drones.isEmpty()) { if (statusLabel != null) statusLabel.setText("Initialize drones first."); return; }
        try {
            double x = clamp(Double.parseDouble(targetXField.getText()), 0, envWidth);
            double y = clamp(Double.parseDouble(targetYField.getText()), 0, envHeight);
            double z = clamp(Double.parseDouble(targetZField.getText()), 0, envDepth);
            setMainTarget(new Vector3(x, y, z));
            if (statusLabel != null) statusLabel.setText("Shared target set. Press Start.");
            renderSim();
        } catch (Exception ex) {
            if (statusLabel != null) statusLabel.setText("Invalid target input.");
        }
    }

    @FXML
    public void randomizeTargetsPerDrone() {
        if (drones.isEmpty()) { if (statusLabel != null) statusLabel.setText("Initialize drones first."); return; }

        perDroneTargetsMode = true;
        mainTarget = null;
        formationRadius = 0.0;
        metricsWritten = false;

        for (Drone drone : drones) {
            double tx = rand.nextDouble() * 80 + 10;
            double ty = rand.nextDouble() * 80 + 10;
            double tz = Math.min(envDepth, rand.nextDouble() * 30 + 5);
            drone.setRtar(new Vector3(tx, ty, tz));
            drone.setVtar(Vector3.zero());
        }

        updateTargetLabel();
        if (statusLabel != null) statusLabel.setText("Random per-drone targets set. Press Start.");
        renderSim();
    }

    @FXML
    public void setTargetForDrone() {
        if (drones.isEmpty()) { if (statusLabel != null) statusLabel.setText("Initialize drones first."); return; }
        try {
            int id = Integer.parseInt(droneIdField.getText().trim());
            if (id < 0 || id >= drones.size()) {
                if (statusLabel != null) statusLabel.setText("Drone ID: 0 to " + (drones.size() - 1));
                return;
            }

            double x = clamp(Double.parseDouble(droneTargetXField.getText()), 0, envWidth);
            double y = clamp(Double.parseDouble(droneTargetYField.getText()), 0, envHeight);
            double z = clamp(Double.parseDouble(droneTargetZField.getText()), 0, envDepth);

            perDroneTargetsMode = true;
            mainTarget = null;
            formationRadius = 0.0;
            metricsWritten = false;

            drones.get(id).setRtar(new Vector3(x, y, z));
            drones.get(id).setVtar(Vector3.zero());

            updateTargetLabel();
            if (statusLabel != null) statusLabel.setText("Target set for Drone " + id);
            renderSim();
        } catch (Exception e) {
            if (statusLabel != null) statusLabel.setText("Invalid per-drone target input.");
        }
    }

    private void setMainTarget(Vector3 t) {
        perDroneTargetsMode = false;
        this.mainTarget = t;
        metricsWritten = false;

        double minDist = minSafeDistance * 1.25;
        formationRadius = Math.max(2.0, (drones.size() * minDist) / (2.0 * Math.PI));

        applyCircleSlotTargets();
        updateTargetLabel();
    }

    private void applyCircleSlotTargets() {
        if (mainTarget == null || drones.isEmpty()) return;

        double z = clamp(mainTarget.z, 0, envDepth);

        for (int i = 0; i < drones.size(); i++) {
            double ang = (2.0 * Math.PI * i) / drones.size();
            double dx = formationRadius * Math.cos(ang);
            double dy = formationRadius * Math.sin(ang);

            double tx = clamp(mainTarget.x + dx, 0, envWidth);
            double ty = clamp(mainTarget.y + dy, 0, envHeight);

            drones.get(i).setRtar(new Vector3(tx, ty, z));
            drones.get(i).setVtar(Vector3.zero());
        }
    }

    private void updateTargetLabel() {
        if (targetLabel == null) return;

        if (!perDroneTargetsMode && mainTarget != null) {
            targetLabel.setText(String.format("Target: (%.2f, %.2f, %.2f)", mainTarget.x, mainTarget.y, mainTarget.z));
        } else if (perDroneTargetsMode) {
            targetLabel.setText("Target: Per-drone targets");
        } else {
            targetLabel.setText("Target: (not set)");
        }
    }

    // ===================== SIM CONTROLS =====================
    @FXML
    public void startSimulation() {
        if (drones.isEmpty()) { if (statusLabel != null) statusLabel.setText("Initialize drones first."); return; }
        if (!perDroneTargetsMode && mainTarget == null) { if (statusLabel != null) statusLabel.setText("Set target first."); return; }
        running = true;
        if (statusLabel != null) statusLabel.setText("Running...");
    }

    @FXML
    public void pauseSimulation() {
        running = false;
        if (statusLabel != null) statusLabel.setText("Paused");
    }

    @FXML
    public void resetSimulation() {
        running = false;
        finished = false;
        simTime = 0.0;
        accumulator = 0;
        lastNs = 0;
        metricsWritten = false;

        followEnabled = false;
        followedDroneId = null;
        updateFollowLabel();

        if (simulator != null) simulator.reset();
        if (statusLabel != null) statusLabel.setText("Reset done.");
        renderSim();
    }

    private void resetAllStateHard() {
        running = false;
        finished = false;
        simTime = 0;
        accumulator = 0;
        lastNs = 0;
        metricsWritten = false;

        drones.clear();
        mainTarget = null;
        formationRadius = 0;
        perDroneTargetsMode = false;

        followEnabled = false;
        followedDroneId = null;
        updateFollowLabel();
    }

    // ===================== SIM ENGINE BUILD =====================
    private void rebuildSimulator() {
        Controller controller = new Controller(kp, kdCtrl, kR, kOmega, mass);
        controller.setDesiredRotation(Matrix3D.identity());
        controller.setDesiredAngularVelocity(Vector3.zero());

        FormationManager formationManager = new FormationManager(formKp, formKv);
        CollisionAvoidance collisionAvoidance = new CollisionAvoidance(kRep, minSafeDistance);
        CommunicationModule communicationModule = new CommunicationModule(commRange, packetLoss);
        Environment environment = new Environment(envWidth, envHeight, envDepth);

        logger = new Logger(positionsCsv, metricsTxt, minSafeDistance);
        communicationModule.setLogger(logger);

        simulator = new Simulator(drones, controller, formationManager, collisionAvoidance,
                communicationModule, environment, logger, dt);
    }

    // ===================== MAIN LOOP =====================
    private void setupLoop() {
        loop = new AnimationTimer() {
            @Override
            public void handle(long now) {
                if (lastNs == 0) { lastNs = now; return; }

                double frameDt = (now - lastNs) / 1_000_000_000.0;
                lastNs = now;

                // ===== SIM step =====
                double speed = (speedSlider != null) ? speedSlider.getValue() : 1.0;

                if (running && !finished && simulator != null) {
                    accumulator += frameDt * speed;

                    while (accumulator >= dt && !finished) {
                        stepSimulationOnce();
                        accumulator -= dt;
                        simTime += dt;

                        if (simTime >= totalTime) {
                            finished = true;
                            running = false;

                            if (!metricsWritten && logger != null) {
                                try { logger.finalize(drones); metricsWritten = true; }
                                catch (Exception ignored) {}
                            }
                            if (statusLabel != null) statusLabel.setText(String.format("Finished (t=%.2f) | Metrics saved", simTime));
                        }
                    }
                }

                // follow camera after sim step
                followCameraStep();

                // ===== GAME update =====
                updateGame(frameDt);

                // fireworks update
                updateFireworks(frameDt);

                // render both
                renderSim();
                renderGame();
            }
        };
        loop.start();
    }

    private void stepSimulationOnce() {
        try {
            simulator.step();
        } catch (Exception ex) {
            running = false;
            if (statusLabel != null) statusLabel.setText("Simulation error: " + ex.getMessage());
            ex.printStackTrace();
        }
    }

    // ===================== RENDER SIM =====================
    private void renderSim() {
        if (mapCanvas == null) return;
        GraphicsContext g = mapCanvas.getGraphicsContext2D();
        double W = mapCanvas.getWidth();
        double H = mapCanvas.getHeight();

        drawBackground(g, W, H, customBgImage, currentBgStyle);

        if (gridCheck != null && gridCheck.isSelected()) drawGrid(g, W, H);
        if (linksCheck != null && linksCheck.isSelected()) drawNeighborLinks(g);

        drawTargets(g);

        for (Drone d : drones) drawDrone(g, d);

        // HUD
        g.setFill(Color.color(1, 1, 1, 0.85));
        g.setFont(Font.font("Consolas", 12));
        g.fillText(String.format("t=%.2f/%.2f  drones=%d  zoom=%.2f",
                        simTime, totalTime, drones.size(), zoom),
                12, 18);

        String followTxt = (followEnabled && followedDroneId != null) ? ("Follow: Drone " + followedDroneId) : "Follow: OFF";
        g.fillText(followTxt, 12, 36);
    }

    private void drawGrid(GraphicsContext g, double W, double H) {
        g.setGlobalAlpha(1.0);
        g.setStroke(Color.color(1, 1, 1, 0.08));
        g.setLineWidth(1);
        for (double wx = 0; wx <= envWidth; wx += 10) {
            double sx = worldToScreenX(wx);
            g.strokeLine(sx, 0, sx, H);
        }
        for (double wy = 0; wy <= envHeight; wy += 10) {
            double sy = worldToScreenY(wy);
            g.strokeLine(0, sy, W, sy);
        }
    }

    private void drawNeighborLinks(GraphicsContext g) {
        g.setStroke(Color.color(1.0, 0.2, 1.0, 0.30));
        g.setLineWidth(2);
        for (int i = 0; i < drones.size(); i++) {
            for (int j = i + 1; j < drones.size(); j++) {
                Vector3 a = drones.get(i).getPosition();
                Vector3 b = drones.get(j).getPosition();
                if (a.distance(b) <= commRange) {
                    g.strokeLine(worldToScreenX(a.x), worldToScreenY(a.y),
                            worldToScreenX(b.x), worldToScreenY(b.y));
                }
            }
        }
    }

    private void drawTargets(GraphicsContext g) {
        g.setGlobalAlpha(1.0);

        if (!perDroneTargetsMode && mainTarget != null) {
            double sx = worldToScreenX(mainTarget.x);
            double sy = worldToScreenY(mainTarget.y);

            g.setStroke(Color.color(1, 0.2, 0.2, 0.9));
            g.setLineWidth(3);
            g.strokeOval(sx - 14, sy - 14, 28, 28);

            g.setFill(Color.color(1, 0.2, 0.2, 0.25));
            g.fillOval(sx - 14, sy - 14, 28, 28);
        } else if (perDroneTargetsMode) {
            g.setStroke(Color.color(1, 0.2, 0.2, 0.7));
            g.setLineWidth(2);
            for (Drone d : drones) {
                Vector3 t = d.getRtar();
                double sx = worldToScreenX(t.x);
                double sy = worldToScreenY(t.y);
                g.strokeOval(sx - 9, sy - 9, 18, 18);
            }
        }
    }

    private void drawDrone(GraphicsContext g, Drone d) {
        Vector3 p = d.getPosition();
        Vector3 v = d.getVelocity();

        double sx = worldToScreenX(p.x);
        double sy = worldToScreenY(p.y);

        // ✅ Tilt now clearly visible (uses zoom)
        double tilt = (tiltSlider != null) ? tiltSlider.getValue() : 0.55;
        sy -= (p.z * tilt * zoom * 0.035);

        double baseSize = 18;
        double size = baseSize * (zoom / 6.5);

        // altitude scaling
        double z01 = clamp(p.z / envDepth, 0, 1);
        double zScale = (1.07 - 0.28 * z01) + (0.06 * (1.0 - z01));
        size *= zScale;

        double yaw = Math.atan2(v.y, v.x);

        // shadow changes with height
        double shadowAlpha = 0.50 - 0.30 * z01;
        double shadowW = size * (1.10 + 0.25 * (1 - z01));
        double shadowH = size * (0.40 + 0.18 * (1 - z01));
        g.setGlobalAlpha(shadowAlpha);
        g.setFill(Color.color(0, 0, 0, 0.65));
        g.fillOval(sx - shadowW * 0.5, (sy + 10) - shadowH * 0.5, shadowW, shadowH);

        // Old spray (your preferred)
        if (sprayCheck != null && sprayCheck.isSelected()) {
            maybeDrawSprayOld(g, d, sx, sy, size);
        }

        g.setGlobalAlpha(1.0);

        if (customDroneImage != null) drawImageDrone(g, customDroneImage, sx, sy, size, yaw);
        else drawBuiltInDrone(g, sx, sy, size, yaw, d.getId(), currentDroneStyle);

        if (labelsCheck != null && labelsCheck.isSelected()) {
            g.setFill(Color.color(1, 1, 1, 0.85));
            g.setFont(Font.font("Consolas", 11));
            g.fillText(String.format("(%.1f, %.1f, %.1f)  Drone %d", p.x, p.y, p.z, d.getId()),
                    sx + size * 0.9, sy - size * 0.6);
        }
    }

    private void maybeDrawSprayOld(GraphicsContext g, Drone d, double sx, double sy, double size) {
        Vector3 p = d.getPosition();
        Vector3 target = d.getRtar();
        double dist = p.subtract(target).magnitude();

        boolean spraying = dist < 6.0;
        if (!spraying) return;

        g.setGlobalAlpha(0.65);
        g.setStroke(Color.color(0.35, 0.80, 1.0, 0.70));
        g.setLineWidth(Math.max(1, size * 0.08));

        // cone lines
        for (int i = 0; i < 14; i++) {
            double a = (i - 7) * 0.07;
            double dx = Math.sin(a) * size * 0.55;
            double dy = size * (0.8 + rand.nextDouble() * 0.35);
            g.strokeLine(sx + dx, sy + size * 0.25, sx + dx * 1.35, sy + dy);
        }

        // droplets
        g.setFill(Color.color(0.35, 0.80, 1.0, 0.55));
        for (int i = 0; i < 22; i++) {
            double rx = (rand.nextDouble() - 0.5) * size * 1.1;
            double ry = rand.nextDouble() * size * 1.2;
            g.fillOval(sx + rx, sy + size * 0.35 + ry, 2.0, 2.0);
        }

        g.setGlobalAlpha(1.0);
    }

    private void drawImageDrone(GraphicsContext g, Image img, double sx, double sy, double size, double yaw) {
        double w = img.getWidth();
        double h = img.getHeight();
        if (w <= 0 || h <= 0) return;

        double aspect = w / h;
        double drawW = size * 1.8;
        double drawH = drawW / aspect;

        g.save();
        g.translate(sx, sy);
        g.rotate(Math.toDegrees(yaw) + 90);
        g.drawImage(img, -drawW / 2.0, -drawH / 2.0, drawW, drawH);
        g.restore();
    }

    private void drawBuiltInDrone(GraphicsContext g, double sx, double sy, double size, double yaw, int id, String style) {
        g.save();
        g.translate(sx, sy);
        g.rotate(Math.toDegrees(yaw) + 90);

        Color body = pickColor(id);

        switch (style) {
            case "🛸 UFO":       drawUfo(g, size, body); break;
            case "👻 Ghost":     drawGhost(g, size); break;
            case "🦜 Parrot":    drawParrot(g, size); break;
            case "🚀 Rocket":    drawRocket(g, size, body); break;
            case "⚡ Lightning": drawLightning(g, size); break;
            case "🐝 Bee":       drawBee(g, size); break;
            case "🐉 Dragon":    drawDragon(g, size); break;
            case "🛰️ Satellite": drawSatellite(g, size, body); break;
            case "🦇 Bat":       drawBat(g, size); break;
            case "🐙 OctoDrone": drawOctoDrone(g, size, body); break;
            default:             drawQuadcopter(g, size, body); break;
        }

        g.restore();
    }

    // ===================== SHAPES =====================

    private void drawQuadcopter(GraphicsContext g, double s, Color body) {
        g.setStroke(Color.color(0.9, 0.9, 0.9, 0.9));
        g.setLineWidth(Math.max(2, s * 0.12));
        g.strokeLine(-s * 0.7, 0, s * 0.7, 0);
        g.strokeLine(0, -s * 0.7, 0, s * 0.7);

        g.setFill(body);
        g.fillRoundRect(-s * 0.35, -s * 0.35, s * 0.70, s * 0.70, 10, 10);

        g.setFill(Color.color(0, 0, 0, 0.7));
        g.fillOval(-s * 0.95, -s * 0.12, s * 0.34, s * 0.24);
        g.fillOval(s * 0.61, -s * 0.12, s * 0.34, s * 0.24);
        g.fillOval(-s * 0.12, -s * 0.95, s * 0.24, s * 0.34);
        g.fillOval(-s * 0.12, s * 0.61, s * 0.24, s * 0.34);

        g.setFill(Color.color(1, 1, 1, 0.85));
        g.fillOval(-s * 0.10, -s * 0.55, s * 0.20, s * 0.20);
    }

    private void drawUfo(GraphicsContext g, double s, Color body) {
        g.setFill(Color.color(0.85, 0.9, 1.0, 0.35));
        g.fillOval(-s * 0.35, -s * 0.65, s * 0.70, s * 0.55);

        g.setFill(body);
        g.fillOval(-s * 0.85, -s * 0.20, s * 1.70, s * 0.65);

        g.setStroke(Color.color(1, 1, 1, 0.35));
        g.setLineWidth(2);
        g.strokeOval(-s * 0.85, -s * 0.20, s * 1.70, s * 0.65);

        for (int i = 0; i < 7; i++) {
            double x = -s * 0.65 + i * (s * 0.22);
            g.setFill(Color.color(1.0, 1.0, 0.6, 0.65));
            g.fillOval(x, -s * 0.03, s * 0.12, s * 0.12);
        }
    }

    private void drawGhost(GraphicsContext g, double s) {
        g.setFill(Color.color(1, 1, 1, 0.75));
        g.fillRoundRect(-s * 0.45, -s * 0.55, s * 0.90, s * 0.95, 18, 18);
        g.fillOval(-s * 0.45, -s * 0.75, s * 0.90, s * 0.55);

        g.setFill(Color.color(0, 0, 0, 0.55));
        g.fillOval(-s * 0.18, -s * 0.40, s * 0.12, s * 0.18);
        g.fillOval(s * 0.06, -s * 0.40, s * 0.12, s * 0.18);
    }

    private void drawParrot(GraphicsContext g, double s) {
        g.setFill(Color.color(0.15, 0.85, 0.35, 0.95));
        g.fillOval(-s * 0.45, -s * 0.35, s * 0.90, s * 0.70);

        g.setFill(Color.color(0.95, 0.2, 0.2, 0.9));
        g.fillOval(-s * 0.15, -s * 0.55, s * 0.30, s * 0.35);

        g.setFill(Color.color(0.98, 0.75, 0.12, 0.95));
        g.fillPolygon(new double[]{0, s * 0.18, 0}, new double[]{-s * 0.30, -s * 0.18, -s * 0.06}, 3);

        g.setFill(Color.color(0, 0, 0, 0.6));
        g.fillOval(-s * 0.05, -s * 0.40, s * 0.10, s * 0.10);
    }

    private void drawRocket(GraphicsContext g, double s, Color body) {
        g.setFill(body);
        g.fillRoundRect(-s * 0.25, -s * 0.70, s * 0.50, s * 1.10, 16, 16);

        g.setFill(Color.color(1, 1, 1, 0.8));
        g.fillOval(-s * 0.10, -s * 0.40, s * 0.20, s * 0.22);

        g.setFill(Color.color(0.25, 0.25, 0.25, 0.9));
        g.fillPolygon(new double[]{-s * 0.25, -s * 0.45, -s * 0.25}, new double[]{s * 0.10, s * 0.32, s * 0.32}, 3);
        g.fillPolygon(new double[]{ s * 0.25,  s * 0.45,  s * 0.25}, new double[]{s * 0.10, s * 0.32, s * 0.32}, 3);

        g.setFill(Color.color(1.0, 0.45, 0.0, 0.75));
        g.fillOval(-s * 0.18, s * 0.35, s * 0.36, s * 0.30);
    }

    private void drawLightning(GraphicsContext g, double s) {
        g.setFill(Color.color(1.0, 0.95, 0.2, 0.95));
        double[] x = {-s*0.15, s*0.10, -s*0.05, s*0.20, -s*0.10, s*0.05};
        double[] y = {-s*0.70, -s*0.25, -s*0.25, s*0.10, s*0.10, s*0.65};
        g.fillPolygon(x, y, x.length);
        g.setStroke(Color.color(0,0,0,0.25));
        g.setLineWidth(2);
        g.strokePolygon(x, y, x.length);
    }

    private void drawBee(GraphicsContext g, double s) {
        g.setFill(Color.color(1.0, 0.85, 0.15, 0.95));
        g.fillOval(-s*0.45, -s*0.25, s*0.90, s*0.50);

        g.setStroke(Color.color(0,0,0,0.7));
        g.setLineWidth(Math.max(2, s*0.10));
        g.strokeLine(-s*0.20, -s*0.22, -s*0.20, s*0.22);
        g.strokeLine(0, -s*0.25, 0, s*0.25);
        g.strokeLine(s*0.20, -s*0.22, s*0.20, s*0.22);

        g.setFill(Color.color(1,1,1,0.35));
        g.fillOval(-s*0.15, -s*0.55, s*0.35, s*0.35);
        g.fillOval(-s*0.25, -s*0.52, s*0.35, s*0.35);

        g.setFill(Color.color(0,0,0,0.6));
        g.fillOval(s*0.30, -s*0.08, s*0.10, s*0.10);
    }

    private void drawDragon(GraphicsContext g, double s) {
        g.setFill(Color.color(0.15, 0.85, 0.55, 0.95));
        g.fillRoundRect(-s*0.40, -s*0.35, s*0.80, s*0.70, 18, 18);

        g.setFill(Color.color(0.95, 0.35, 0.1, 0.95));
        g.fillPolygon(new double[]{-s*0.10, 0, s*0.10}, new double[]{-s*0.55, -s*0.35, -s*0.55}, 3);

        g.setFill(Color.color(1,1,1,0.8));
        g.fillOval(-s*0.10, -s*0.10, s*0.20, s*0.20);
        g.setFill(Color.color(0,0,0,0.7));
        g.fillOval(-s*0.04, -s*0.04, s*0.08, s*0.08);
    }

    private void drawSatellite(GraphicsContext g, double s, Color body) {
        g.setFill(body);
        g.fillRoundRect(-s*0.20, -s*0.20, s*0.40, s*0.40, 10, 10);

        g.setFill(Color.color(0.25, 0.6, 1.0, 0.85));
        g.fillRoundRect(-s*0.90, -s*0.25, s*0.55, s*0.50, 8, 8);
        g.fillRoundRect(s*0.35, -s*0.25, s*0.55, s*0.50, 8, 8);

        g.setStroke(Color.color(1,1,1,0.5));
        g.setLineWidth(2);
        g.strokeLine(-s*0.35, 0, -s*0.05, 0);
        g.strokeLine(s*0.05, 0, s*0.35, 0);
    }

    private void drawBat(GraphicsContext g, double s) {
        g.setFill(Color.color(0.1, 0.1, 0.1, 0.95));
        g.fillPolygon(
                new double[]{-s*0.8, -s*0.4, -s*0.1, 0, s*0.1, s*0.4, s*0.8, s*0.3, 0, -s*0.3},
                new double[]{0, -s*0.25, -s*0.10, -s*0.35, -s*0.10, -s*0.25, 0, s*0.15, s*0.05, s*0.15},
                10
        );
        g.setFill(Color.color(1,1,1,0.6));
        g.fillOval(-s*0.08, -s*0.22, s*0.06, s*0.06);
        g.fillOval(s*0.02, -s*0.22, s*0.06, s*0.06);
    }

    private void drawOctoDrone(GraphicsContext g, double s, Color body) {
        g.setFill(body);
        g.fillOval(-s*0.30, -s*0.30, s*0.60, s*0.60);

        g.setStroke(Color.color(1,1,1,0.7));
        g.setLineWidth(Math.max(2, s*0.10));
        for (int i = 0; i < 8; i++) {
            double a = i * (Math.PI * 2.0 / 8.0);
            g.strokeLine(0,0, Math.cos(a)*s*0.70, Math.sin(a)*s*0.70);
        }

        g.setFill(Color.color(0,0,0,0.55));
        for (int i = 0; i < 8; i++) {
            double a = i * (Math.PI * 2.0 / 8.0);
            g.fillOval(Math.cos(a)*s*0.78 - s*0.09, Math.sin(a)*s*0.78 - s*0.07, s*0.18, s*0.14);
        }
    }

    private Color pickColor(int id) {
        Color[] palette = new Color[] {
                Color.web("#22c55e"), Color.web("#60a5fa"), Color.web("#f97316"),
                Color.web("#a78bfa"), Color.web("#f43f5e"), Color.web("#14b8a6"),
                Color.web("#eab308"), Color.web("#fb7185"), Color.web("#38bdf8")
        };
        return palette[Math.floorMod(id, palette.length)];
    }

    // ===================== BACKGROUND (shared) =====================
    private void drawBackground(GraphicsContext g, double W, double H, Image custom, String style) {
        if (custom != null) {
            double iw = custom.getWidth(), ih = custom.getHeight();
            if (iw > 0 && ih > 0) {
                double scale = Math.max(W / iw, H / ih);
                double dw = iw * scale, dh = ih * scale;
                double dx = (W - dw) / 2.0, dy = (H - dh) / 2.0;
                g.setGlobalAlpha(1.0);
                g.drawImage(custom, dx, dy, dw, dh);
                g.setFill(Color.color(0, 0, 0, 0.25));
                g.fillRect(0, 0, W, H);
                return;
            }
        }

        switch (style) {
            case "🏜️ Desert":
                g.setFill(Color.web("#c2a56b")); g.fillRect(0, 0, W, H);
                g.setFill(Color.color(0.6, 0.45, 0.2, 0.15));
                for (int i = 0; i < 90; i++) g.fillOval((i * 97) % W, (i * 53) % H, 220, 90);
                break;

            case "🌊 Ocean":
                g.setFill(Color.web("#0b3a66")); g.fillRect(0, 0, W, H);
                g.setFill(Color.color(1, 1, 1, 0.05));
                for (int i = 0; i < 120; i++) g.fillRoundRect((i * 71) % W, (i * 37) % H, 160, 24, 18, 18);
                break;

            case "🌃 City Night":
                g.setFill(Color.web("#0b1020")); g.fillRect(0, 0, W, H);
                g.setFill(Color.color(1, 1, 0.6, 0.12));
                for (int i = 0; i < 200; i++) g.fillRect((i * 41) % W, (i * 29) % H, 2, 2);
                break;

            case "🌌 Space":
                g.setFill(Color.web("#050814")); g.fillRect(0, 0, W, H);
                g.setFill(Color.color(1, 1, 1, 0.14));
                for (int i = 0; i < 250; i++) g.fillOval((i * 53) % W, (i * 97) % H, 2.2, 2.2);
                break;

            case "☁️ Sky":
                g.setFill(Color.web("#7dd3fc")); g.fillRect(0, 0, W, H);
                g.setFill(Color.color(1, 1, 1, 0.25));
                for (int i = 0; i < 25; i++) g.fillOval((i * 140) % W, (i * 70) % H, 240, 90);
                break;

            default: // Farm Field
                g.setFill(Color.web("#14532d")); g.fillRect(0, 0, W, H);
                g.setFill(Color.color(0, 0, 0, 0.12));
                for (int i = -40; i < 120; i++) g.fillRect(0, i * 18, W, 6);
                g.setFill(Color.color(1, 1, 1, 0.04));
                for (int i = 0; i < 40; i++) g.fillRect(i * 60, 0, 10, H);
                break;
        }
    }

    // ===================== GAME: UI BUTTON HANDLERS =====================
    @FXML
    public void startGameMission() {
        gameRunning = true;
        gameMissionCompleted = false;
        victoryPopupShown = false;
        sprayHoldSeconds = 0.0;
        if (gameTarget == null) gameRandomTarget();
        if (gameInfoLabel != null) gameInfoLabel.setText("Status: Mission started! Use WASD to move. SPACE to spray.");
        if (gameCanvas != null) gameCanvas.requestFocus();
    }

    @FXML
    public void gameRandomTarget() {
        gameTarget = new Vector3(
                clamp(rand.nextDouble() * 90 + 5, 0, envWidth),
                clamp(rand.nextDouble() * 90 + 5, 0, envHeight),
                2
        );
        sprayHoldSeconds = 0.0;
        gameMissionCompleted = false;
        victoryPopupShown = false;
        if (gameInfoLabel != null) gameInfoLabel.setText("Status: New target spawned! Get inside circle + spray 10s.");
        if (gameCanvas != null) gameCanvas.requestFocus();
    }

    @FXML
    public void resetGameMission() {
        gameRunning = false;
        gameMissionCompleted = false;
        victoryPopupShown = false;
        sprayHoldSeconds = 0.0;
        fireworks.clear();
        gameVel = Vector3.zero();
        gamePos = new Vector3(10, 10, 2);
        if (gameInfoLabel != null) gameInfoLabel.setText("Status: Reset. Press Start Mission.");
        if (gameCanvas != null) gameCanvas.requestFocus();
    }

    // ===================== GAME: UPDATE =====================
    private void updateGame(double frameDt) {
        if (!gameRunning || gameCanvas == null) return;
        if (frameDt <= 0) return;

        // WASD -> acceleration
        double ax = 0, ay = 0;
        if (keysDown.contains(KeyCode.W)) ay -= manualAccel;
        if (keysDown.contains(KeyCode.S)) ay += manualAccel;
        if (keysDown.contains(KeyCode.A)) ax -= manualAccel;
        if (keysDown.contains(KeyCode.D)) ax += manualAccel;

        // integrate velocity + drag
        gameVel = new Vector3(
                gameVel.x + ax * frameDt,
                gameVel.y + ay * frameDt,
                0
        );
        gameVel = new Vector3(
                gameVel.x - gameVel.x * manualDrag * frameDt,
                gameVel.y - gameVel.y * manualDrag * frameDt,
                0
        );

        // integrate position
        gamePos = new Vector3(
                clamp(gamePos.x + gameVel.x * frameDt, 0, envWidth),
                clamp(gamePos.y + gameVel.y * frameDt, 0, envHeight),
                2
        );

        // SPACE spray
        boolean spaceDown = keysDown.contains(KeyCode.SPACE);
        boolean insideTarget = false;

        if (gameTarget != null) {
            double dx = gamePos.x - gameTarget.x;
            double dy = gamePos.y - gameTarget.y;
            insideTarget = (dx*dx + dy*dy) <= (gameTargetRadius * gameTargetRadius);
        }

        if (spaceDown && insideTarget && !gameMissionCompleted) {
            sprayHoldSeconds += frameDt;
            if (sprayHoldSeconds >= missionSprayRequired) {
                gameMissionCompleted = true;
                spawnVictoryFireworks();
                showMissionCompletedPopupOnce();
                if (gameInfoLabel != null) gameInfoLabel.setText("Status: ✅ Mission completed!");
            } else {
                if (gameInfoLabel != null) {
                    gameInfoLabel.setText(String.format("Status: Spraying... %.1f / %.1fs", sprayHoldSeconds, missionSprayRequired));
                }
            }
        } else {
            // don’t decay progress; you can change this if you want it to drop when leaving
            if (!gameMissionCompleted && gameInfoLabel != null) {
                double dist = (gameTarget == null) ? 0 : Math.hypot(gamePos.x - gameTarget.x, gamePos.y - gameTarget.y);
                gameInfoLabel.setText(String.format("Status: Target dist %.2f | Spray %.1f/%.1fs",
                        dist, sprayHoldSeconds, missionSprayRequired));
            }
        }
    }

    // ===================== GAME: RENDER =====================
    private void renderGame() {
        if (gameCanvas == null) return;
        GraphicsContext g = gameCanvas.getGraphicsContext2D();
        double W = gameCanvas.getWidth();
        double H = gameCanvas.getHeight();

        drawBackground(g, W, H, gameBgImage, gameBgStyle);

        // simple grid for game
        g.setStroke(Color.color(1,1,1,0.06));
        g.setLineWidth(1);
        for (double wx = 0; wx <= envWidth; wx += 10) {
            double sx = gameWorldToScreenX(wx, W);
            g.strokeLine(sx, 0, sx, H);
        }
        for (double wy = 0; wy <= envHeight; wy += 10) {
            double sy = gameWorldToScreenY(wy, H);
            g.strokeLine(0, sy, W, sy);
        }

        // target circle (BIG)
        if (gameTarget != null) {
            double sx = gameWorldToScreenX(gameTarget.x, W);
            double sy = gameWorldToScreenY(gameTarget.y, H);

            double rpx = (gameTargetRadius * (W / envWidth));
            double rpy = (gameTargetRadius * (H / envHeight));
            double rr = Math.min(rpx, rpy);

            g.setStroke(Color.color(1, 0.2, 0.2, 0.95));
            g.setLineWidth(3);
            g.strokeOval(sx - rr, sy - rr, rr * 2, rr * 2);

            g.setFill(Color.color(1, 0.2, 0.2, 0.18));
            g.fillOval(sx - rr, sy - rr, rr * 2, rr * 2);
        }

        // drone
        double dx = gameWorldToScreenX(gamePos.x, W);
        double dy = gameWorldToScreenY(gamePos.y, H);

        // draw spray if SPACE
        boolean spaceDown = keysDown.contains(KeyCode.SPACE);
        if (spaceDown && gameTarget != null && !gameMissionCompleted) {
            // use same old spray look (downwards cone)
            g.setGlobalAlpha(0.65);
            g.setStroke(Color.color(0.35, 0.80, 1.0, 0.70));
            g.setLineWidth(2);
            for (int i = 0; i < 14; i++) {
                double a = (i - 7) * 0.07;
                double ddx = Math.sin(a) * 18 * 0.55;
                double ddy = 18 * (0.8 + rand.nextDouble() * 0.35);
                g.strokeLine(dx + ddx, dy + 6, dx + ddx * 1.35, dy + ddy);
            }
            g.setFill(Color.color(0.35, 0.80, 1.0, 0.55));
            for (int i = 0; i < 22; i++) {
                double rx = (rand.nextDouble() - 0.5) * 18 * 1.1;
                double ry = rand.nextDouble() * 18 * 1.2;
                g.fillOval(dx + rx, dy + 6 + ry, 2.0, 2.0);
            }
            g.setGlobalAlpha(1.0);
        }

        // drone icon (game)
        if (gameDroneImage != null) {
            drawImageDrone(g, gameDroneImage, dx, dy, 18, 0);
        } else {
            drawBuiltInDrone(g, dx, dy, 18, 0, 0, gameDroneStyle);
        }

        // HUD top-left
        g.setFill(Color.color(1,1,1,0.85));
        g.setFont(Font.font("Consolas", 13));
        g.fillText(String.format("Pos: (%.1f, %.1f)  Vel: (%.1f, %.1f)",
                gamePos.x, gamePos.y, gameVel.x, gameVel.y), 12, 18);

        if (gameTarget != null) {
            double dist = Math.hypot(gamePos.x - gameTarget.x, gamePos.y - gameTarget.y);
            g.fillText(String.format("Target dist: %.2f   Spray: %.1f/%.1fs",
                    dist, sprayHoldSeconds, missionSprayRequired), 12, 38);
        } else {
            g.fillText("Target: (not set)", 12, 38);
        }

        // fireworks (after win)
        drawFireworks(g);
    }

    private double gameWorldToScreenX(double wx, double W) { return (wx / envWidth) * W; }
    private double gameWorldToScreenY(double wy, double H) { return (wy / envHeight) * H; }

    // ===================== FIREWORKS =====================
    private void spawnVictoryFireworks() {
        fireworks.clear();
        // burst in center of game screen space (we’ll draw in screen coords)
        for (int i = 0; i < 180; i++) {
            double angle = rand.nextDouble() * Math.PI * 2;
            double speed = 80 + rand.nextDouble() * 220;
            double vx = Math.cos(angle) * speed;
            double vy = Math.sin(angle) * speed;
            double life = 0.8 + rand.nextDouble() * 0.9;
            Color c = Color.hsb(rand.nextDouble() * 360, 0.9, 1.0, 1.0);
            fireworks.add(new Firework(0,0,vx,vy,life,c));
        }
    }

    private void updateFireworks(double dt) {
        if (fireworks.isEmpty()) return;
        double gravity = 120;
        for (int i = fireworks.size() - 1; i >= 0; i--) {
            Firework f = fireworks.get(i);
            f.life -= dt;
            if (f.life <= 0) { fireworks.remove(i); continue; }
            f.vy += gravity * dt;
            f.x += f.vx * dt;
            f.y += f.vy * dt;
        }
    }

    private void drawFireworks(GraphicsContext g) {
        if (fireworks.isEmpty() || gameCanvas == null) return;
        double W = gameCanvas.getWidth();
        double H = gameCanvas.getHeight();

        double cx = W * 0.5;
        double cy = H * 0.35;

        for (Firework f : fireworks) {
            double a = clamp(f.life / 1.2, 0, 1);
            g.setGlobalAlpha(0.9 * a);
            g.setFill(Color.color(f.c.getRed(), f.c.getGreen(), f.c.getBlue(), 1.0));
            g.fillOval(cx + f.x, cy + f.y, 3, 3);
        }
        g.setGlobalAlpha(1.0);

        if (gameMissionCompleted) {
            g.setFill(Color.color(1,1,1,0.9));
            g.setFont(Font.font("Consolas", 32));
            g.fillText("MISSION COMPLETED!", W*0.5 - 190, H*0.5);
        }
    }

    private void showMissionCompletedPopupOnce() {
        if (victoryPopupShown) return;
        victoryPopupShown = true;

        Platform.runLater(() -> {
            Alert a = new Alert(Alert.AlertType.INFORMATION);
            a.setTitle("Mission Completed!");
            a.setHeaderText("🎉 You win!");
            a.setContentText("You sprayed inside the target for 10 seconds.");
            a.show();
        });
    }

    // ===================== UTILITIES =====================
    private int safeParseInt(String s, int fallback) {
        try { return Integer.parseInt(s.trim()); }
        catch (Exception e) { return fallback; }
    }

    private double clamp(double v, double lo, double hi) {
        if (v < lo) return lo;
        if (v > hi) return hi;
        return v;
    }

    private double worldToScreenX(double wx) { return panX + wx * zoom; }
    private double worldToScreenY(double wy) { return panY + wy * zoom; }
    private double screenToWorldX(double sx) { return (sx - panX) / zoom; }
    private double screenToWorldY(double sy) { return (sy - panY) / zoom; }

}
