import math
import serial
import pynmea2

def angle_to_target_radius(latA_rad,lonA_rad,latB_rad,lonB_rad):

    #Δφ = ln( tan( latB / 2 + π / 4 ) / tan( latA / 2 + π / 4) )
    latA_rad = math.radians(A["Lat"])
    latB_rad = math.radians(B["Lat"])
    lonA_rad = math.radians(A["Lon"])
    lonB_rad = math.radians(B["Lon"])
    print("latA_rad :",latA_rad,"lonA_rad :",lonA_rad)
    long_delta = lonB_rad - lonA_rad
    # Calcul de y
    y = math.sin(long_delta) * math.cos(latB_rad)
    # Calcul de x
    x = (math.cos(latA_rad) * math.sin(latB_rad)
         - math.sin(latA_rad) * math.cos(latB_rad) * math.cos(long_delta))
    # Calcul de l'angle
    angle = math.degrees(math.atan2(y, x))
    while angle < 0:
        angle =angle+360
    print( "angle C", angle)
    return(angle)
            
def distance_to_target_meter(A,B):
    #distance(A, B) = R * arccos(sin(lata)sin * (LATB)+ cos(lata)cos *(LATB)cos* (Lona - lonB))

    #R=radius in Km
    R = 6372.795477598 #Km
    distanceAB= R   *   math.acos(
        math.sin(latA_rad)*
        math.sin(latB_rad)+
        math.cos(latA_rad)*
        math.cos(latB_rad)*
        math.cos(lonA_rad-lonB_rad))

    print("Distance (m) :" + str(distanceAB/1000))
    return(distanceAB/1000)
            
def Pos_weel(latA_rad,lonA_rad, d,Tau):
    # R=radius in Km
    R = 6372.795477598  # Km
    
    
    
    #latB = asin( sin( latA) * cos( d / R ) + cos( latA ) * sin( d / R ) * cos( θ ))
    Lat_Weel= math.asin(latA_rad * math.cos(d/R))+math.cos(latA_rad)*math.sin(d/R)*math.cos(Tau)
    #lonB = lonA + atan2(sin( θ ) * sin( d / R ) * cos( latA ), cos( d / R ) − sin( latA ) * sin( latB ))
    Lon_Weel = lonA_rad + math.atan2(math.sin(Tau) * math.sin(d / R) * math.cos(latA_rad), math.cos(d / R) - math.sin(latA_rad)*math.sin(Lat_Weel))

    return [Lat_Weel,Lon_Weel]


    
    
    
    
    
