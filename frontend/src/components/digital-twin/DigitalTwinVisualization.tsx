import React, { useState, useEffect } from 'react';

interface DigitalTwinProps {
  robotModel?: string;
  simulationState?: any;
}

const DigitalTwinVisualization: React.FC<DigitalTwinProps> = ({
  robotModel = 'humanoid_robot_1',
  simulationState = {}
}) => {
  const [isSimulating, setIsSimulating] = useState(false);
  const [robotPosition, setRobotPosition] = useState({ x: 0, y: 0, z: 0 });
  const [sensorData, setSensorData] = useState<any>({});

  useEffect(() => {
    // Simulate updating robot position and sensor data
    if (isSimulating) {
      const interval = setInterval(() => {
        setRobotPosition(prev => ({
          x: prev.x + (Math.random() - 0.5) * 0.1,
          y: prev.y + (Math.random() - 0.5) * 0.1,
          z: 0
        }));

        setSensorData({
          camera: `Image captured at (${robotPosition.x.toFixed(2)}, ${robotPosition.y.toFixed(2)})`,
          lidar: `Distance readings: ${(Math.random() * 10).toFixed(2)}m`,
          imu: `Orientation: ${(Math.random() * 360).toFixed(2)}°`
        });
      }, 1000);

      return () => clearInterval(interval);
    }
  }, [isSimulating, robotPosition.x, robotPosition.y]);

  const toggleSimulation = () => {
    setIsSimulating(!isSimulating);
  };

  return (
    <div className="digital-twin-container">
      <div className="simulation-controls">
        <button onClick={toggleSimulation} className={`btn ${isSimulating ? 'btn-danger' : 'btn-success'}`}>
          {isSimulating ? 'Stop Simulation' : 'Start Simulation'}
        </button>
      </div>

      <div className="simulation-display">
        <div className="robot-visualization">
          <h3>Robot: {robotModel}</h3>
          <div className="position-data">
            <p>X: {robotPosition.x.toFixed(2)}</p>
            <p>Y: {robotPosition.y.toFixed(2)}</p>
            <p>Z: {robotPosition.z.toFixed(2)}</p>
          </div>
        </div>

        <div className="sensor-data">
          <h4>Sensor Data:</h4>
          <div className="sensor-item">
            <strong>Camera:</strong> {sensorData.camera || 'No data'}
          </div>
          <div className="sensor-item">
            <strong>LiDAR:</strong> {sensorData.lidar || 'No data'}
          </div>
          <div className="sensor-item">
            <strong>IMU:</strong> {sensorData.imu || 'No data'}
          </div>
        </div>
      </div>

      <div className="simulation-status">
        <p>Status: {isSimulating ? 'Running' : 'Stopped'}</p>
        <p>Fidelity Level: High</p>
      </div>
    </div>
  );
};

export default DigitalTwinVisualization;