import socket
import json
import time

import pygame
from pygame.locals import *

from OpenGL.GL import *
from OpenGL.GLU import *


verticies = (
    (1, -1, -0.25),
    (1, 1, -0.25),
    (-1, 1, -0.25),
    (-1, -1, -0.25),
    (1, -1, 0.25),
    (1, 1, 0.25),
    (-1, -1, 0.25),
    (-1, 1, 0.25)
)

edges = (
    (0,1),
    (0,3),
    (0,4),
    (2,1),
    (2,3),
    (2,7),
    (6,3),
    (6,4),
    (6,7),
    (5,1),
    (5,4),
    (5,7)
)

def Cube():
    glBegin(GL_LINES)
    glColor3f(0, 255, 0)
    for edge in edges:
        for vertex in edge:
            glVertex3fv(verticies[vertex])
    glEnd()

def Rotate(x, y, z):
    glRotatef(x, 1, 0, 0)
    glRotatef(y, 0, 1, 0)
    glRotatef(z, 0, 0, 1)

def drawText(x, y, textString):     
    font = pygame.font.Font ("vt323.ttf", 20)
    textSurface = font.render(textString, True, (0,255,0,255), (0,0,0,0))     
    textData = pygame.image.tostring(textSurface, "RGBA", True)     
    glRasterPos2f(x, y)     
    glDrawPixels(textSurface.get_width(), textSurface.get_height(), GL_RGBA, GL_UNSIGNED_BYTE, textData)

def main():
    x = 0
    y = 0
    z = 0
    elapsed = 0

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    print("Trying to connect to SckzQuad...")

    while True:
        try:
            s.connect(('192.168.128.223', 2000))
            break
        except Exception as e:
            print(f"Connection failed, retrying in 1 second:\n{e}")
            time.sleep(1)

    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF|OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.1, 50.0)

    glTranslatef(0.0,0.0, -5)


    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()


        try:
            bytestr = s.recv(128)
            string = bytestr.decode("UTF-8")
            data = json.loads(string)

            if data['type'] == "heading":
                x = data['x']
                y = data['y']
                z = data['z']
                elapsed = data['elapsed']

        except Exception as e:
            print(f"There was an issue:\n{e}")


        glPushMatrix()
        
        Rotate(x, y, z)
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        Cube()

        glPopMatrix()

        drawText(-2, 1.5, f"It took {elapsed} seconds to poll the sensor")

        pygame.display.flip()

main()

