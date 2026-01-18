import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import DigitalTwinVisualization from '../../components/digital-twin/DigitalTwinVisualization';

export default function DigitalTwin() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout title={`${siteConfig.title}`} description="Digital Twin Module: Gazebo & Unity Integration">
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <h1>Digital Twin: Gazebo & Unity Integration</h1>
            <p>This module teaches simulation-first development of humanoid robots using Gazebo and Unity integrated with ROS 2.</p>

            <h2>Interactive Demo</h2>
            <DigitalTwinVisualization robotModel="Humanoid Robot Simulator" />

            <h2>Module Structure</h2>
            <ol>
              <li><a href="#digital-twin-fundamentals">Digital Twin Fundamentals</a></li>
              <li><a href="#physics-simulation">Physics Simulation Basics</a></li>
              <li><a href="#gazebo-simulation">Gazebo for Robotics Simulation</a></li>
              <li><a href="#sensor-simulation">Sensor Simulation</a></li>
              <li><a href="#unity-visualization">Unity Visualization</a></li>
              <li><a href="#ros2-integration">ROS 2 Integration</a></li>
              <li><a href="#simulation-workflow">Simulation-First Workflow</a></li>
            </ol>

            <section id="digital-twin-fundamentals">
              <h3>Digital Twin Fundamentals</h3>
              <p>A digital twin is a virtual replica of a physical system that enables real-time monitoring, simulation, and optimization. In robotics, digital twins serve as testing environments for algorithms before physical deployment.</p>
            </section>

            <section id="physics-simulation">
              <h3>Physics Simulation Basics</h3>
              <p>Understanding rigid body dynamics, gravity, collisions, and time stepping in simulation environments.</p>
            </section>

            <section id="gazebo-simulation">
              <h3>Gazebo for Robotics Simulation</h3>
              <p>Setting up and running Gazebo simulations for humanoid robots with physics modeling and sensor simulation.</p>
            </section>

            <section id="sensor-simulation">
              <h3>Sensor Simulation</h3>
              <p>Simulating cameras, LiDAR, and IMUs in virtual environments with realistic noise models.</p>
            </section>

            <section id="unity-visualization">
              <h3>Unity Visualization</h3>
              <p>Creating Unity scenes for 3D robotic model visualization and interaction.</p>
            </section>

            <section id="ros2-integration">
              <h3>ROS 2 Integration</h3>
              <p>Connecting simulation environments to the ROS 2 communication framework.</p>
            </section>

            <section id="simulation-workflow">
              <h3>Simulation-First Workflow</h3>
              <p>Following a simulation-first approach to robot development and testing.</p>
            </section>
          </div>
        </div>
      </div>
    </Layout>
  );
}