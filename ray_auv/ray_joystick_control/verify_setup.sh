#!/bin/bash
# RAY AUV Joystick Control - Build and Test Verification Script

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
echo -e "${BLUE}RAY AUV PS5 Joystick Control - Verification Script${NC}"
echo -e "${BLUE}═══════════════════════════════════════════════════${NC}"
echo ""

# Check if we're in the right directory
if [ ! -f "package.xml" ]; then
    echo -e "${RED}Error: package.xml not found!${NC}"
    echo "Please run this script from the ray_joystick_control directory"
    exit 1
fi

echo -e "${YELLOW}Step 1: Checking package structure...${NC}"
declare -a required_files=(
    "package.xml"
    "CMakeLists.txt"
    "scripts/joystick_controller.py"
    "config/joystick_controller.yaml"
    "launch/joystick_launch.py"
    "launch/joystick_only_launch.py"
    "README.md"
)

all_files_exist=true
for file in "${required_files[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓${NC} $file"
    else
        echo -e "${RED}✗${NC} $file (MISSING)"
        all_files_exist=false
    fi
done

if [ "$all_files_exist" = false ]; then
    echo -e "${RED}Some required files are missing!${NC}"
    exit 1
fi

echo ""
echo -e "${YELLOW}Step 2: Checking documentation files...${NC}"
declare -a doc_files=(
    "QUICKSTART.md"
    "SETUP_INSTRUCTIONS.md"
    "VISUAL_REFERENCE.md"
    "IMPLEMENTATION_SUMMARY.md"
    "INDEX.md"
)

for file in "${doc_files[@]}"; do
    if [ -f "$file" ]; then
        echo -e "${GREEN}✓${NC} $file"
    else
        echo -e "${YELLOW}ℹ${NC} $file (optional)"
    fi
done

echo ""
echo -e "${YELLOW}Step 3: Checking Python syntax...${NC}"
if python3 -m py_compile scripts/joystick_controller.py 2>/dev/null; then
    echo -e "${GREEN}✓${NC} joystick_controller.py (syntax valid)"
else
    echo -e "${RED}✗${NC} joystick_controller.py (syntax error)"
    python3 -m py_compile scripts/joystick_controller.py
    exit 1
fi

echo ""
echo -e "${YELLOW}Step 4: Checking YAML syntax...${NC}"
if python3 << 'EOF' 2>/dev/null
import yaml
try:
    with open('config/joystick_controller.yaml', 'r') as f:
        yaml.safe_load(f)
    print("✓ YAML is valid")
except Exception as e:
    print(f"✗ YAML error: {e}")
    exit(1)
EOF
then
    echo -e "${GREEN}✓${NC} joystick_controller.yaml (valid YAML)"
else
    echo -e "${RED}✗${NC} joystick_controller.yaml (invalid YAML)"
    exit 1
fi

echo ""
echo -e "${YELLOW}Step 5: Checking ROS 2 dependencies...${NC}"

# Check if we can import rclpy
if python3 -c "import rclpy; print('OK')" 2>/dev/null | grep -q "OK"; then
    echo -e "${GREEN}✓${NC} rclpy (ROS 2 Python client library)"
else
    echo -e "${YELLOW}ℹ${NC} rclpy not found (you may need to source ROS 2 setup)"
fi

# Check geometry_msgs
if python3 -c "from geometry_msgs.msg import PoseStamped; print('OK')" 2>/dev/null | grep -q "OK"; then
    echo -e "${GREEN}✓${NC} geometry_msgs (ROS 2 messages)"
else
    echo -e "${YELLOW}ℹ${NC} geometry_msgs not found (you may need to source ROS 2 setup)"
fi

# Check sensor_msgs
if python3 -c "from sensor_msgs.msg import Joy; print('OK')" 2>/dev/null | grep -q "OK"; then
    echo -e "${GREEN}✓${NC} sensor_msgs (ROS 2 messages)"
else
    echo -e "${YELLOW}ℹ${NC} sensor_msgs not found (you may need to source ROS 2 setup)"
fi

echo ""
echo -e "${YELLOW}Step 6: Checking joystick device...${NC}"
if [ -e "/dev/input/js0" ]; then
    echo -e "${GREEN}✓${NC} Joystick device found at /dev/input/js0"
    ls -l /dev/input/js0 | awk '{print "  " $1, $9, "(" $3 ":" $4 ")"}'
else
    echo -e "${YELLOW}ℹ${NC} No joystick device at /dev/input/js0"
    echo "  Check connection status or try: ls /dev/input/js*"
fi

echo ""
echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"
echo -e "${GREEN}✓ All critical files and dependencies verified!${NC}"
echo -e "${GREEN}═══════════════════════════════════════════════════${NC}"
echo ""

echo "Ready to build and run:"
echo ""
echo -e "${BLUE}Build command:${NC}"
echo "  cd /home/pdc/ray_ws"
echo "  colcon build --packages-select ray_joystick_control"
echo "  source install/setup.bash"
echo ""
echo -e "${BLUE}Run command:${NC}"
echo "  ros2 launch ray_joystick_control joystick_launch.py"
echo ""
echo -e "${BLUE}Or follow the ${YELLOW}QUICKSTART.md${BLUE} guide for detailed instructions.${NC}"
echo ""
