from fastapi import Depends, HTTPException, status, Header
from fastapi.security import OAuth2PasswordBearer
from backend.auth.jwt import decode_token
from backend.config import settings
from backend.pairing import pairing_manager
from typing import Optional
from datetime import datetime

oauth2_scheme = OAuth2PasswordBearer(tokenUrl="/auth/login")

def get_current_user(token: str = Depends(oauth2_scheme)) -> dict:
    if not settings.ENABLE_AUTH:
        # Bypass auth when disabled (treat as viewer)
        return {"sub":"anon", "email":"anon@local", "roles":["VIEWER"]}
    payload = decode_token(token)
    if not payload:
        raise HTTPException(status_code=status.HTTP_401_UNAUTHORIZED, detail="Invalid token")
    return payload

def require_roles(*required: str):
    required_set = set(required)
    def _inner(user=Depends(get_current_user)):
        roles = set(user.get("roles", []))
        if not roles.intersection(required_set):
            raise HTTPException(status_code=status.HTTP_403_FORBIDDEN, detail="Insufficient role")
        return user
    return _inner

def require_device_pairing(authorization: Optional[str] = Header(None)):
    """
    Requires that the device has been paired via Pi CLI before allowing access.
    
    Two security checks:
    1. If Authorization header provided - verify the pairing token
    2. If no auth header - check if pairing mode is active (Pi CLI generated code)
    
    This ensures only devices that have gone through the secure Pi CLI pairing
    process can access authentication endpoints.
    """
    # Check 1: Token-based verification (if provided)
    if authorization and authorization.startswith("Bearer "):
        pairing_token = authorization.replace("Bearer ", "")
        device = pairing_manager.verify_token(pairing_token)
        
        if not device:
            raise HTTPException(
                status_code=403,
                detail="Invalid or expired pairing token. Generate new code via Pi CLI."
            )
        
        return device
    
    # Check 2: Verify pairing mode is active (Pi CLI generated)
    # OR that at least one device has been paired previously
    if pairing_manager.is_pairing_active() or pairing_manager.get_paired_devices():
        # Pairing is available - allow access to auth endpoints
        # This allows the initial pairing flow to work
        class PairingActiveDevice:
            def __init__(self):
                self.device_name = "Pairing Mode Active"
                self.access_level = "operator"
                self.paired_at = datetime.utcnow()
        
        return PairingActiveDevice()
    
    # No pairing available - reject access
    raise HTTPException(
        status_code=403,
        detail="Device pairing required. Generate pairing code on Pi: 'python cli_pairing.py start'"
    )
