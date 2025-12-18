#!/bin/bash
set -e

echo "=========================================="
echo "Celestia Ground Station - Installation"
echo "=========================================="
echo ""

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    echo "❌ Error: Do not run this script as root (sudo)"
    echo "   The script will ask for sudo password when needed"
    exit 1
fi

# Configuration
INSTALL_DIR="/home/$USER/celestia-groundstation"
SERVICE_NAME="celestia-backend.service"
PYTHON_MIN_VERSION="3.9"

echo "📦 Installation directory: $INSTALL_DIR"
echo "👤 User: $USER"
echo ""

# Check Python version
echo "🔍 Checking Python version..."
if ! command -v python3 &> /dev/null; then
    echo "❌ Python 3 is not installed"
    echo "   Install with: sudo apt update && sudo apt install -y python3 python3-pip python3-venv"
    exit 1
fi

PYTHON_VERSION=$(python3 -c 'import sys; print(".".join(map(str, sys.version_info[:2])))')
echo "✓ Python $PYTHON_VERSION found"
echo ""

# Create installation directory
echo "📁 Creating installation directory..."
mkdir -p "$INSTALL_DIR"
cd "$INSTALL_DIR"
echo "✓ Created $INSTALL_DIR"
echo ""

# Copy backend files
echo "📋 Copying backend files..."
if [ -d "$OLDPWD/backend" ]; then
    cp -r "$OLDPWD/backend" .
    echo "✓ Backend files copied"
else
    echo "❌ Error: backend directory not found in $OLDPWD"
    exit 1
fi
echo ""

# Create virtual environment
echo "🐍 Creating Python virtual environment..."
cd "$INSTALL_DIR"
python3 -m venv backend/venv
echo "✓ Virtual environment created"
echo ""

# Install dependencies
echo "📦 Installing Python dependencies..."
source backend/venv/bin/activate
pip install --upgrade pip
if [ -f "backend/requirements.txt" ]; then
    pip install -r backend/requirements.txt
    echo "✓ Dependencies installed"
else
    echo "⚠️  No requirements.txt found, skipping dependency installation"
fi
deactivate
echo ""

# Install systemd service
echo "⚙️  Installing systemd service..."
sudo cp "$OLDPWD/deployment/$SERVICE_NAME" /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable $SERVICE_NAME
echo "✓ Systemd service installed and enabled"
echo ""

# Start the service
echo "🚀 Starting Celestia backend..."
sudo systemctl start $SERVICE_NAME
sleep 3
echo ""

# Check service status
echo "📊 Checking service status..."
if sudo systemctl is-active --quiet $SERVICE_NAME; then
    echo "✅ Service is running!"
else
    echo "❌ Service failed to start. Checking logs..."
    sudo journalctl -u $SERVICE_NAME -n 20 --no-pager
    exit 1
fi
echo ""

# Create desktop shortcut
echo "🖥️  Creating desktop shortcut..."
DESKTOP_FILE="$HOME/Desktop/celestia-admin.desktop"
cat > "$DESKTOP_FILE" << DESKTOP_EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=Celestia Admin Panel
Comment=Ground Station Administration
Exec=xdg-open https://localhost:8000/admin
Icon=network-server
Terminal=false
Categories=Network;
DESKTOP_EOF

chmod +x "$DESKTOP_FILE"
echo "✓ Desktop shortcut created"
echo ""

# Display info
HOSTNAME=$(hostname)
LOCAL_IP=$(hostname -I | awk '{print $1}')

echo "=========================================="
echo "✅ Installation Complete!"
echo "=========================================="
echo ""
echo "📍 Service Information:"
echo "   • Status: sudo systemctl status $SERVICE_NAME"
echo "   • Logs:   sudo journalctl -u $SERVICE_NAME -f"
echo "   • Stop:   sudo systemctl stop $SERVICE_NAME"
echo "   • Start:  sudo systemctl start $SERVICE_NAME"
echo ""
echo "🌐 Access Admin Panel:"
echo "   • Local:  https://localhost:8000/admin"
echo "   • Network: https://$LOCAL_IP:8000/admin"
echo ""
echo "🔐 Next Steps:"
echo "   1. Open the admin panel on this device"
echo "   2. Click 'Start Pairing Mode'"
echo "   3. Share the 6-digit code with remote users"
echo "   4. Remote users visit: https://ui.celestiaenergy.com"
echo ""
echo "⚠️  Note: Accept the self-signed SSL certificate warning"
echo "    This is normal for local HTTPS connections"
echo ""
