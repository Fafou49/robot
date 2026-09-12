"""Live camera stream for Raspberry Pi #1 (robot side) -- see stream_server.py
for the implementation. Independent from the NMEA control link (link/): the
web server project (robot-webserver, Raspberry Pi #2) proxies
http://<this Pi's IP>:CAMERA_PORT/stream.mjpg from its own /media/camera
route, so the live feed stays behind the site's login instead of being
exposed directly on the LAN.
"""
