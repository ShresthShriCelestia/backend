# Quick Start Guide - For Teammate in Prague

## What You Need to Do

### Step 1: Install Cloudflared (5 minutes)

Run these commands on the Raspberry Pi:

```bash
# Download cloudflared
curl -L --output cloudflared.deb \
  https://github.com/cloudflare/cloudflared/releases/latest/download/cloudflared-linux-arm64.deb

# Install it
sudo dpkg -i cloudflared.deb

# Login (this will open a browser)
cloudflared tunnel login
```

**IMPORTANT:** The `cloudflared tunnel login` command will print a URL. Send me that URL so I can authorize it from the UK.

### Step 2: Create Tunnel

After I authorize it, run:

```bash
# Create tunnel
cloudflared tunnel create celestia-groundstation

# Note the tunnel ID that gets printed!
```

**Send me the tunnel ID** - it looks like: `abc123def-456g-789h-ijk0-lmnopqrstuv`

### Step 3: That's It!

I'll handle the rest remotely via SSH tunnel:
- Install ground station software
- Configure tunnel  
- Set up auto-start
- Test everything

## What You'll Have After Setup

1. **Backend Auto-Starts**: Ground station software runs automatically on boot
2. **Admin Panel**: Desktop shortcut to open admin panel
3. **Remote Access**: I can SSH in and help troubleshoot anytime

## Troubleshooting

### Can't run cloudflared?
Make sure you're in the Pi's terminal (not SSH yet)

### Browser won't open for login?
- Check if Pi is connected to internet: `ping google.com`
- Try running: `cloudflared tunnel login` again

### Need help?
Message me the error message and I'll help!

## After Installation

To start pairing mode:
1. Double-click "Celestia Admin Panel" desktop icon
2. Click "Start Pairing Mode" button
3. Share the 6-digit code with remote operators

That's it! 🚀
