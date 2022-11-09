from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
import time

DRONE_NUM = 6
board = [[0 for k in range(DRONE_NUM * 3)], [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 0, 0, 0, 0, 0, 0]]


class Drone:
    def __init__(self, x, id):
        self.x = x
        
    def openWays(self):
        w = []
        ways = [self.x]
        if self.x-1 >= 0:
            ways.append(self.x-1)
        if self.x+1 < DRONE_NUM*3:
            ways.append(self.x+1)
        for way in ways:
            if board[1][way] < 2:
                w.append(way)
        return w
    def waysLeft(self):
        w = []
        for d in drones[:drones.index(self)]:
            for way in d.openWays():
                if way not in w:
                    w.append(way)
        return len(w)
    def waysRight(self):
        w = []
        for d in drones[drones.index(self) +1:]:
            for way in d.openWays():
                if way not in w:
                    w.append(way)
        return len(w)
    def move(self):
        if len(self.openWays()) > 0:
            self.x = self.openWays()[0]
        else:
            if self.waysRight() > self.waysLeft():
                self.moveNextRight()
            else:
                self.moveNextLeft()
    def moveNextRight(self):
        while True:
            if (drones.index(self) < len(drones)-1 and drones[drones.index(self)+1].x == self.x+1):
                drones[drones.index(self)+1].moveNextRight()
            self.x += 1
            if board[1][self.x] == 0:
                break
    def moveNextLeft(self):
        while True:
            if (drones.index(self) > 0 and drones[drones.index(self)-1].x == self.x-1):
                drones[drones.index(self)-1].moveNextLeft()
            self.x -= 1
            if board[1][self.x] == 0: 
                break

def animate():
    shownBoard.set_data(board)
    return shownBoard

def updateBoard():
    for i in range(len(board)):
        for j in range(len(board[i])):
            if board[i][j] == 1:
                board[i][j] = 0
    for d in drones:
        if board[0][d.x] > 1:
            print("FAIL")
        else:
            board[0][d.x] = 1
    shownBoard.set_data(board)
    plt.draw()
    
def getDrone(x):
    for d in drones:
        if d.x == x:
            return d

#plt.show()


shownBoard = plt.imshow(board)

drones = []
for i in range(DRONE_NUM):
    drones.append(Drone( i*3+1, i))

updateBoard()

for d in drones:
    d.move()

updateBoard()
plt.show()



