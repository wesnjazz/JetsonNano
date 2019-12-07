from jetbot import Camera

width = 640
height = 480
camera = Camera.instance(width=width, height=height)

while True:
    print(camera.value[width//2][height//2][0])
