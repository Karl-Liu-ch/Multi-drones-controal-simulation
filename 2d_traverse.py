from matplotlib import pyplot as plt
from matplotlib import animation
import numpy as np
import time
import random

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

    def moveRight(self):
        if (drones.index(self) < len(drones)-1 and drones[drones.index(self)+1].x == self.x+1):
            drones[drones.index(self)+1].moveRight()
        self.x += 1

    def moveLeft(self):
        if (drones.index(self) > 0 and drones[drones.index(self)-1].x == self.x-1):
            drones[drones.index(self)-1].moveLeft()
        self.x -= 1

    def reset(self):
        while True:
            if self.x == drones.index(self)*3 + 1:
                break
            if self.x < drones.index(self)*3 + 1:
                self.moveRight()
            elif self.x > drones.index(self)*3 + 1:
                self.moveLeft()

def animate(i):
    if i%4 == 0:
        for d in drones:
            d.move()
    if i%4 == 1:
        moveObs()
    if i%4 == 2:
        moveObs()
        newObs()        
    if i%4 == 3:
        for d in drones:
            d.reset()
    updateBoard()

    return [shownBoard]

def newObs():
    obs_num = random.randint(0, len(board[0]) - len(drones))
    obstacles = random.sample(range(len(board[1])), obs_num)
    for o in obstacles:
        board[1][o] = 2

def moveObs():
    for i in range(len(board[0])):
        if board[0][i] > 1:
            board[0][i] = 0
        if board[1][i] > 0:
            board[0][i] = board[1][i]
            board[1][i] = 0
        

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
    
def getDrone(x):
    for d in drones:
        if d.x == x:
            return d



drones = []
for i in range(DRONE_NUM):
    drones.append(Drone( i*3+1, i))

fig = plt.figure()
shownBoard = plt.imshow(board)
#plt.show()
anim = animation.FuncAnimation(fig, animate, frames = 20, interval = 200, blit = True)
print("done")
plt.show()



