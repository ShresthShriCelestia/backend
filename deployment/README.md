# Celestia Ground Station - Deployment Guide

Complete guide for deploying the ground station backend on Raspberry Pi.

## Overview

The ground station system consists of:
- **Backend**: FastAPI server running on Raspberry Pi (auto-starts on boot)
- **Admin Panel**: Local web interface for pairing and status (https://localhost:8000/admin)
- **Remote UI**: Cloud-hosted frontend at https://ui.celestiaenergy.com

## Prerequisites

### Hardware
- Raspberry Pi 4 or newer (2GB+ RAM recommended)
- SD card (16GB+ recommended)
- Network connection (WiFi or Ethernet)

### Software
- Raspberry Pi OS (64-bit recommended)
- Python 3.9 or newer
- Internet connection (for initial setup)

## Installation

### 1. Prepare Raspberry Pi

```bash
# Update system
sudo apt update && sudo apt upgrade -y

# Install required packages
sudo apt install -y python3 python3-pip python3-venv git

# Optional: Set hostname (helps identify the station)
sudo hostnamectl set-hostname celestia-groundstation
```

### 2. Copy Files to Raspberry Pi

Transfer the entire `WebUI` directory to the Pi:

**Option A: Using git**
```bash
git clone https://github.com/your-repo/WebUI.git
cd WebUI
```

**Option B: Using scp (from your computer)**
```bash
scp -r ./WebUI pi@<PI_IP_ADDRESS>:~/
```

### 3. Run Installation Script

```bash
cd WebUI
chmod +x deployment/install.sh
./deployment/install.sh
```

The script will:
- ✅ Create installation directory at `/home/pi/celestia-groundstation`
- ✅ Set up Python virtual environment
- ✅ Install dependencies
- ✅ Configure systemd service (auto-start on boot)
- ✅ Start the backend
- ✅ Create desktop shortcut

## Accessing the Admin Panel

### On the Raspberry Pi
1. Open desktop
2. Double-click "Celestia Admin Panel" icon
3. Or visit: https://localhost:8000/admin

### From Another Device on Same Network
Visit: `https://<PI_IP_ADDRESS>:8000/admin`

**Note:** You'll see an SSL certificate warning - this is normal. Click "Advanced" → "Proceed to site"

## Pairing Remote Users

### On the Ground Station (Raspberry Pi)
1. Open admin panel
2. Click "**Start Pairing Mode**"
3. Note the **6-digit pairing code**
4. Share this code with remote users

### Remote Users
1. Visit https://ui.celestiaenergy.com
2. Enter ground station address: `https://<PI_IP_ADDRESS>:8000`
3. Enter the 6-digit pairing code
4. Optionally check "Remember this connection"

## Service Management

### Check Status
```bash
sudo systemctl status celestia-backend.service
```

### View Logs
```bash
# Live logs
sudo journalctl -u celestia-backend.service -f

# Last 50 lines
sudo journalctl -u celestia-backend.service -n 50
```

### Start/Stop/Restart
```bash
# Stop the service
sudo systemctl stop celestia-backend.service

# Start the service
sudo systemctl start celestia-backend.service

# Restart the service
sudo systemctl restart celestia-backend.service
```

### Disable Auto-Start
```bash
sudo systemctl disable celestia-backend.service
```

### Enable Auto-Start
```bash
sudo systemctl enable celestia-backend.service
```

## Cloudflare Tunnel Setup (Optional)

For remote access without port forwarding:

### 1. Install cloudflared
```bash
curl -L --output cloudflared.deb \
  https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64.deb
sudo dpkg -i cloudflared.deb
```

### 2. Authenticate
```bash
cloudflared tunnel login
```
This will open a browser - log in with your Cloudflare account.

### 3. Create Tunnel
```bash
cloudflared tunnel create celestia-groundstation
```
Note the tunnel ID from the output.

### 4. Configure Tunnel
Create `/home/pi/.cloudflared/config.yml`:

```yaml
tunnel: <TUNNEL_ID>
credentials-file: /home/pi/.cloudflared/<TUNNEL_ID>.json

ingress:
  # SSH access
  - hostname: ssh.celestiaenergy.com
    service: ssh://localhost:22
  
  # Backend API
  - hostname: groundstation.celestiaenergy.com
    service: https://localhost:8000
    originRequest:
      noTLSVerify: true
  
  # Catch-all
  - service: http_status:404
```

### 5. Install as Service
```bash
sudo cloudflared service install
sudo systemctl start cloudflared
sudo systemctl enable cloudflared
```

### 6. Configure DNS
In Cloudflare dashboard, add DNS records:
- `ssh.celestiaenergy.com` → CNAME → `<TUNNEL_ID>.cfargotunnel.com`
- `groundstation.celestiaenergy.com` → CNAME → `<TUNNEL_ID>.cfargotunnel.com`

## Troubleshooting

### Backend won't start
```bash
# Check service status
sudo systemctl status celestia-backend.service

# View detailed logs
sudo journalctl -u celestia-backend.service -n 100

# Common issues:
# - Python dependencies missing: Run install script again
# - Port 8000 already in use: Check for other services
# - Permission issues: Ensure files are owned by pi user
```

### Can't access admin panel
- Check firewall: `sudo ufw status`
- Verify service is running: `sudo systemctl status celestia-backend.service`
- Check correct URL: `https://localhost:8000/admin` (note HTTPS)

### SSL certificate errors
This is expected for self-signed certificates. On each device:
1. Visit the backend URL (e.g., https://192.168.1.100:8000)
2. Click "Advanced" or "Show details"
3. Click "Proceed to site" or "Accept the risk"

### Pairing code not working
- Ensure pairing mode is active (code expires after 5 minutes)
- Verify network connectivity between Pi and remote device
- Check that clocks are synchronized (for token validation)

## Uninstallation

```bash
# Stop and disable service
sudo systemctl stop celestia-backend.service
sudo systemctl disable celestia-backend.service
sudo rm /etc/systemd/system/celestia-backend.service
sudo systemctl daemon-reload

# Remove installation directory
rm -rf /home/pi/celestia-groundstation

# Remove desktop shortcut
rm ~/Desktop/celestia-admin.desktop
```

## Security Notes

- Backend uses self-signed SSL certificates by default
- Device pairing provides access control
- User authentication with JWT tokens
- Regular security updates recommended: `sudo apt update && sudo apt upgrade`

## Support

For issues or questions:
- Check logs: `sudo journalctl -u celestia-backend.service -f`
- Review this documentation
- Contact: support@celestiaenergy.com
