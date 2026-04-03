package DroneSimulator;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;

public class Logger {
    
    private final String positionsFile;
    private final String metricsFile;
    private final double minSafeDistance;
    
    private BufferedWriter positionsWriter;
//    private List<String> positionLogs;
    
    private int collisionC;
    
    private int totalMessages;
    private int successfulMessages;
    private int totalSteps;
    
    public Logger(String positionsFile, String metricsFile, double minSafeDistance) {
    	
        this.positionsFile = positionsFile;
        this.metricsFile = metricsFile;
        this.minSafeDistance = minSafeDistance;
//        this.positionLogs = new ArrayList<>();
        this.totalMessages = 0;
        this.successfulMessages = 0;
        this.totalSteps = 0;
//        collisionC = 0;
        
        initializePositionsFile();
    }
   
    private void initializePositionsFile() {
        try {
            positionsWriter = new BufferedWriter(new FileWriter(positionsFile));
            positionsWriter.write("Time,Drone ID,Px,Py,Pz,Vx,Vy,Vz,Tz\n");
            
        } catch (IOException e) {
            System.out.println("Error initializing positions file: " + e.getMessage());
        }
    }
    
    public void log(double time, Drone drone) throws IOException {
        if (positionsWriter == null) {
            return;
        }
        
        Vector3 p = drone.getPosition();
        Vector3 v = drone.getVelocity();
        Vector3 thrust = drone.getThrustForce();
        
        double Tz = thrust.getZ();
        
//        String logEntry = ();
        
        positionsWriter.write(time+","+drone.getId()+","+p.x+","+p.y+","+p.z+","+v.x+","+v.y+","+v.z+","+Tz+"\n");
        totalSteps++;
    }
    
    public void logCommunication(boolean success) {
        totalMessages++;
        if (success) {
            successfulMessages++;
        }
    }
    
    public double computeAverageSpacing(List<Drone> drones) {
        if (drones.size() < 2) {
            return 0.0;
        }
        
        double totalDistance = 0.0;
        int pairCount = 0;
        
        for (int i = 0; i < drones.size(); i++) {
            for (int j = i + 1; j < drones.size(); j++) {
                Vector3 pi = drones.get(i).getPosition();
                Vector3 pj = drones.get(j).getPosition();
                totalDistance += pi.distance(pj);
                pairCount++;
            }
        }
        
        return totalDistance / pairCount;
    }

    
    public double computeCommunicationSuccessRate() {
        if (totalMessages == 0) {
            return 0.0;
        }
        return (double) successfulMessages / totalMessages;
    }
    
    public void finalize(List<Drone> drones) {
        try {
            // Close positions file
            if (positionsWriter != null) {
                positionsWriter.close();
            }
            
            // Write metrics.txt
            writeMetricsFile(drones);
            
            System.out.println("\nLogging completed:");
            System.out.println("Positions saved to: " + positionsFile);
            System.out.println("Metrics saved to: " + metricsFile);
            
        } catch (IOException e) {
            System.out.println("Error finalizing logger: " + e.getMessage());
        }
    }
    
    private void writeMetricsFile(List<Drone> drones) throws IOException {
        BufferedWriter metricsWriter = new BufferedWriter(new FileWriter(metricsFile));
        
        double avgSpacing = computeAverageSpacing(drones);
        double commSuccessRate = computeCommunicationSuccessRate();
        
        // Write metrics
        metricsWriter.write("DRONE SIMULATOR\nTHE FINAL SUMMARY\n\n");
        
        metricsWriter.write("Fleet Statistics:\n");
        metricsWriter.write("Number of Drones: "+ drones.size()+"\n");
        metricsWriter.write("Total Simulation Steps: "+ totalSteps+"\n\n\n");
        
        metricsWriter.write("Coordination Metrics:\n");
        metricsWriter.write("Average Spacing: "+ avgSpacing+"\n");
        metricsWriter.write("Collision Count: "+ collisionC+"\n");
        metricsWriter.write("Minimum Safe Distance: "+ minSafeDistance+"\n\n\n");
        
        metricsWriter.write("Communication Metrics:\n");
        metricsWriter.write("Total Messages: "+ totalMessages+"\n");
        metricsWriter.write("Successful Messages: "+successfulMessages+"\n");
        metricsWriter.write("Communication Success Rate: "+ commSuccessRate+" : " + commSuccessRate * 100+"\n\n\n");
        
        metricsWriter.write("Final Drone Positions:\n");
        for (Drone drone : drones) {
            Vector3 pos = drone.getPosition();
            Vector3 vel = drone.getVelocity();
            Vector3 target = drone.getRtar();
            double distToTarget = pos.subtract(target).magnitude();
            
            metricsWriter.write("  Drone: "+ drone.getId()+"\n");
            metricsWriter.write("  Position: "+pos.toString()+"\n");
            metricsWriter.write("  Velocity: "+vel.toString()+"\n");
            metricsWriter.write("  Target: "+target.toString()+"\n");
            metricsWriter.write("  Distance to Target: "+ distToTarget+"\n");
            metricsWriter.write("  Speed: "+ vel.magnitude()+"\n\n");
        }
        
        metricsWriter.close();
    }
    
    public void reset() {
        try {
            if (positionsWriter != null) {
                positionsWriter.close();
            }
            
            totalMessages = 0;
            successfulMessages = 0;
            totalSteps = 0;
            
            initializePositionsFile();
            
        } catch (IOException e) {
            System.out.println("Error resetting logger: " + e.getMessage());
        }
    }
    
    public int getTotalMessages() {
        return totalMessages;
    }
    
    public int getSuccessfulMessages() {
        return successfulMessages;
    }
    
    public int getTotalSteps() {
        return totalSteps;
    }
    
    public String getPositionsFile() {
        return positionsFile;
    }
    
    public String getMetricsFile() {
        return metricsFile;
    }

	public int getCollisionC() {
		return collisionC;
	}

	public void setCollisionC(int collisionC) {
		this.collisionC += collisionC;
	}
}