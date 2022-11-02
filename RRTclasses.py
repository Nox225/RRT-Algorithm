import pygame
import math
import random

class Board:
    def __init__(self, start, end, boardSize, obstacleSize, obstacleNumber):
        self.start = start
        self.end = end
        self.boardSize = boardSize
        self.boardX, self.boardY = self.boardSize

        self.white = (255, 255, 255)
        self.grey = (62, 63, 75)
        self.green = (77, 232, 93)
        self.red = (255, 0, 0)
        self.blue = (73, 191, 215)

        self.board = pygame.display.set_mode((self.boardX, self.boardY))
        self.board.fill((27, 31, 64))
        self.nodeRad = 0
        self.nodeThickness = 0
        self.edgeThickness = 1

        self.obstacles = []
        self.obstacleSize = obstacleSize
        self.obstacleNumber = obstacleNumber

    def drawBoard(self, obstacles):
        pygame.draw.circle(self.board, self.green, self.start, 10, 0)
        pygame.draw.circle(self.board, self.red, self.end, 10, 0)
        self.spawnObstacles(obstacles)

    def drawPath(self, path):
        for point in path:
            pygame.draw.circle(self.board, (0, 0, 255), point, 6, 0)

    def spawnObstacles(self, obstacles):
        for i in range(0, len(obstacles)):
            pygame.draw.rect(self.board, self.grey, obstacles[i])

class Graph:
    def __init__(self, start, end, boardSize, obstacleSize, obstacleNumber):
        (x, y) = start
        self.start = start
        self.end = end
        self.pathFound = False
        self.boardX, self.boardY = boardSize
        self.x = []
        self.y = []
        self.parent = []

        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        obstacles = []
        self.obstacleSize = obstacleSize
        self.obstacleNumber = obstacleNumber

        self.goalState = None
        self.path = []

    def getRandomSquarePos(self):
        x = int(random.uniform(0, self.boardX-self.obstacleSize))
        y = int(random.uniform(0, self.boardY-self.obstacleSize))

        return (x, y)

    def createObstacles(self):
        obs = []
        for i in range(0, self.obstacleNumber):
            square = None
            isColliding = True
            while isColliding:
                square = pygame.Rect(
                    self.getRandomSquarePos(),
                    (self.obstacleSize*random.random(), self.obstacleSize*random.random()))

                if square.collidepoint(self.start) or square.collidepoint(self.end):
                   isColliding = True
                else:
                    isColliding = False
            obs.append(square)
        
        walls = [pygame.Rect((500, 0), (30, 300)), 
                 pygame.Rect((500, 600), (30, 300)),
                 pygame.Rect((0, 270), (400, 30)),
                 pygame.Rect((0, 600), (200, 30)),
                 pygame.Rect((300, 600), (200, 30)),
                 pygame.Rect((1000, 0), (30, 250)),
                 pygame.Rect((1000, 350), (30, 250)),
                 pygame.Rect((1000, 600), (300, 30)),
                 pygame.Rect((1400, 600), (300, 30)),
                 pygame.Rect((1000, 600), (30, 100)),
                 pygame.Rect((1000, 800), (30, 150))
                ]
        
        for i in range(0, len(walls)):
            obs.append(walls[i])

        self.obstacles = obs.copy()
        return obs

    def addNode(self, n, x, y):
        self.x.insert(n, x)
        self.y.insert(n, y)

    # def removeNode(self, n):
    #     self.x.pop(n)
    #     self.y.pop(n)

    def addEdge(self, parent, child):
        self.parent.insert(child, parent)

    # def removeEdge(self, n):
    #     self.parent.pop(n)

    # def numberOfNodes(self):
    #     return len(self.x)

    def dist(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])

        return ((x1-x2)**2 + (y1-y2)**2)**(0.5)

    def randomPoint(self):
        x = int(random.uniform(0, self.boardX))
        y = int(random.uniform(0, self.boardY))
        
        return x, y

    def nearest(self, n):
        dmin = self.dist(0, n)
        nearId = 0
        for i in range(0, n):
            if self.dist(i, n) < dmin:
                dmin = self.dist(i, n)
                nearId = i
        return nearId

    def isFree(self):
        id = len(self.x) - 1
        (x, y) = (self.x[id], self.y[id])
        for i in range(0, len(self.obstacles)):
            if self.obstacles[i].collidepoint(x, y):
                self.x.pop(id)
                self.y.pop(id)
                return False
        return True

    def crossObstacle(self, x1, y1, x2, y2):
        for i in range(0, len(self.obstacles)):
            for j in range(0, 100):
                d = j/100
                x = x1*d + x2*(1-d)
                y = y1*d + y2*(1-d)
                if self.obstacles[i].collidepoint(x, y):
                    return True
        return False

    def join(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])

        if self.crossObstacle(x1, y1, x2, y2):
            self.x.pop(n2)
            self.y.pop(n2)
            return False
        else:
            self.addEdge(n1, n2)
            return True

    def step(self, nearestNode, randomNode):
        (nearestX, nearestY) = (self.x[nearestNode], self.y[nearestNode])
        (randomX, randomY) = (self.x[randomNode], self.y[randomNode])

        if self.dist(nearestNode, randomNode) > 35:
            deltaX = randomX - nearestX
            deltaY = randomY - nearestY
            theta = math.atan2(deltaY, deltaX)
            (x, y) = (math.cos(theta) * 35 + nearestX, math.sin(theta) * 35 + nearestY)

            self.x.pop(randomNode)
            self.y.pop(randomNode)

            if abs(x - self.end[0]) < 35 and abs(y - self.end[1]) < 35:
                self.addNode(randomNode, self.end[0], self.end[1])
                self.goalState = randomNode
                self.pathFound = True
            else:
                self.addNode(randomNode, x, y)

    def pathToGoal(self):
        if self.pathFound:
            self.path = []
            self.path.append(self.goalState)
            prevPos = self.parent[self.goalState]
            while (prevPos != 0):
                self.path.append(prevPos)
                prevPos = self.parent[prevPos]
            self.path.append(0)
        return self.pathFound

    def getPathXY(self):
        pathXY = []
        for point in self.path:
            (x, y) = (self.x[point], self.y[point])
            pathXY.append((x, y))
        return pathXY

    def bias(self, end):
        nodes = len(self.x)
        self.addNode(nodes, end[0], end[1])
        nearest = self.nearest(nodes)
        self.step(nearest, nodes)
        self.join(nearest, nodes)

        return self.x, self.y, self.parent

    def expand(self):
        nodes = len(self.x)
        x, y = self.randomPoint()
        self.addNode(nodes, x, y)
        if self.isFree():
            nearest = self.nearest(nodes)
            self.step(nearest, nodes)
            self.join(nearest, nodes)

        return self.x, self.y, self.parent

    def cost(self):
        pass




def main():
    size = (1600, 900)
    start = (50, 50)
    end = (1400, 800)
    obstacleSize = 80
    obstacleNumber = 80

    pygame.init()
    board = Board(start, end, size, obstacleSize, obstacleNumber)
    graph = Graph(start, end, size, obstacleSize, obstacleNumber)

    obstacles = graph.createObstacles()
    board.drawBoard(obstacles)

    # while(True):
    #     x, y = graph.randomPoint()
    #     n = graph.numberOfNodes()
    #     graph.addNode(n, x, y)
    #     graph.addEdge(n-1, n)
    #     x1, y1 = graph.x[n], graph.y[n]
    #     x2, y2 = graph.x[n-1], graph.y[n-1]
    #     if graph.isFree():
    #         pygame.draw.circle(board.board, board.red, (graph.x[n], graph.y[n]), 2, 0)
    #         if not graph.crossObstacle(x1, y1, x2, y2):
    #             pygame.draw.line(board.board, board.red, (x1, y1), (x2, y2), 2)

    #     pygame.display.update()
        # pygame.event.clear()
        # pygame.event.wait(0)

    # while(iteration < 1000):
    #     if iteration % 10 == 0:
    #         X, Y, Parent = graph.bias(end)
    #         # pygame.draw.circle(board.board, board.grey, (X[-1], Y[-1]), 2, 0)
    #         pygame.draw.line(board.board, board.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), 2)
    #     else:
    #         X, Y, Parent = graph.expand()
    #         # pygame.draw.circle(board.board, board.grey, (X[-1], Y[-1]), 2, 0)
    #         pygame.draw.line(board.board, board.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), 2)

    #     if iteration % 5 == 0:
    #         pygame.display.update()

    #     iteration += 1

    # pygame.display.update()
    # pygame.event.clear()
    # pygame.event.wait(0)
    iteration = 0

    while(not graph.pathToGoal()):
        if iteration % 2000 == 0:
            X, Y, Parent = graph.bias(end)
            # pygame.draw.circle(board.board, board.grey, (X[-1], Y[-1]), 2, 0)
            pygame.draw.line(board.board, board.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), 2)
        else:
            X, Y, Parent = graph.expand()
            # pygame.draw.circle(board.board, board.grey, (X[-1], Y[-1]), 2, 0)
            pygame.draw.line(board.board, board.blue, (X[-1], Y[-1]), (X[Parent[-1]], Y[Parent[-1]]), 2)

        if iteration % 5 == 0:
            pygame.display.update()

        iteration += 1

    board.drawPath(graph.getPathXY())
    pygame.display.update()
    pygame.event.clear()
    pygame.event.wait(0)



if __name__ == '__main__':
    result = False
    while not result:
        try:
            main()
            result = True
        except:
            result = False