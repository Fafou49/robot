"""Network link between Raspberry Pi #2 (web server) and Raspberry Pi #1
(robot): NMEA-inspired sentence format (link.nmea) and the TCP server that
receives/dispatches those sentences on the robot side (link.server).

See pages/protocole_controle.html on the web server project for the full
sentence reference (this module implements exactly that format).
"""
