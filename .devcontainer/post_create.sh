#!/bin/bash
set -e

cd /workspaces

# PX4 Autopilot
if [ ! -d "PX4-Autopilot" ]; then
    git clone -b release/1.14 https://github.com/PX4/PX4-Autopilot.git --recursive
    cp /workspaces/walls.sdf /workspaces/PX4-Autopilot/Tools/simulation/gz/worlds/
fi

cd offboard_ws/src

# PX4 Messages
if [ ! -d "px4_msgs" ]; then
    git clone -b release/1.14 https://github.com/PX4/px4_msgs.git --recursive
fi

# scripts

mkdir -p /scripts

# Start the DDS bridge
cat > /scripts/run_dds_agent.sh << 'EOF'
#!/bin/bash
MicroXRCEAgent udp4 -p 8888
EOF
chmod +x /scripts/run_dds_agent.sh
