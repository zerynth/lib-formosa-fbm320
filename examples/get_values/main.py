################################################################################
# Barometer Sensor Example
#
# Created: 2020-08-26
# Author: S. Torneo
#
################################################################################

import streams
from formosa.fbm320 import fbm320

streams.serial()

try:
    # Setup sensor 
    print("start...")
    fbm = fbm320.FBM320(I2C0)
    print("Ready!")
    print("--------------------------------------------------------")
except Exception as e:
    print("Error: ",e)

try:
    while True:
        temp, press, altitude = fbm.get_values()
        print("Temperature: ", temp, "C")
        print("Pressure: ", press, "hPa")
        print("Altitude: ", altitude, "m")
        print("--------------------------------------------------------")
        sleep(5000)
except Exception as e:
    print("Error2: ",e)

