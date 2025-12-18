import jwt
from jwt import PyJWTError
from datetime import datetime, timedelta
from typing import Optional, Dict, Any
from backend.config import settings


def create_access_token(sub: str, email: str, roles: list[str]) -> str:
    now = datetime.utcnow()
    payload = {
        "sub": sub,
        "email": email,
        "roles": roles,
        "iat": int(now.timestamp()),
        "exp": int((now + timedelta(minutes=settings.ACCESS_TOKEN_MIN)).timestamp()),
    }
    return jwt.encode(payload, settings.JWT_SECRET, algorithm=settings.JWT_ALG)


def decode_token(token: str) -> Optional[Dict[str, Any]]:
    """Decode a JWT token and return the payload or None.

    This helper is tolerant of common variants: it will strip a leading
    "Bearer " prefix if present, and returns None on any decode/validation
    error.
    """
    if not token:
        return None

    # If caller passed an Authorization header value, strip the scheme.
    if token.startswith("Bearer "):
        token = token.split(" ", 1)[1]

    try:
        payload = jwt.decode(token, settings.JWT_SECRET, algorithms=[settings.JWT_ALG])
        # Ensure minimal expected fields exist
        if not isinstance(payload, dict) or "sub" not in payload:
            return None
        return payload
    except PyJWTError as e:
        # Optional: log debug info to help diagnose token problems in the terminal
        try:
            print(f"[auth.jwt] token decode error: {type(e).__name__}: {e}")
        except Exception:
            pass
        return None
