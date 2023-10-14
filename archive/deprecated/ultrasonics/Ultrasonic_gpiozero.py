from gpiozero import DistanceSensor

sensor = DistanceSensor(echo=18, trigger=17)
print('Distance to nearest object is', sensor.distance * 100, 'cm')
